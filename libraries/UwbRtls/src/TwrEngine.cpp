#include "TwrEngine.h"

// ---- static members --------------------------------------------------------
TwrEngine*      TwrEngine::_instance     = nullptr;
volatile bool   TwrEngine::_sentFlag     = false;
volatile bool   TwrEngine::_receivedFlag = false;

void TwrEngine::onSent()     { _sentFlag = true; }
void TwrEngine::onReceived() { _receivedFlag = true; }

// ---- setup -----------------------------------------------------------------
void TwrEngine::begin(TwrRole role, uint8_t myAddr, uint16_t antennaDelay,
                      uint8_t pinSs, uint8_t pinIrq, uint8_t pinRst) {
  _role     = role;
  _myAddr   = myAddr;
  _antDelay = antennaDelay;
  _instance = this;

  DW1000.begin(pinIrq, pinRst);
  DW1000.select(pinSs);
  configure();

  DW1000.attachSentHandler(TwrEngine::onSent);
  DW1000.attachReceivedHandler(TwrEngine::onReceived);
  DW1000.attachReceiveFailedHandler(TwrEngine::onReceiveFailed);
  DW1000.attachReceiveTimeoutHandler(TwrEngine::onReceiveTimeout);

  startRx();
}

void TwrEngine::configure() {
  DW1000.newConfiguration();
  DW1000.setDefaults();
  DW1000.setDeviceAddress(_myAddr);
  DW1000.setNetworkId(UWB_NETWORK_ID);
  DW1000.enableMode(UWB_RADIO_MODE);
  DW1000.setChannel(UWB_CHANNEL);
  // Antenna delay is cached and written to the chip by commitConfiguration().
  DW1000.setAntennaDelay(_antDelay);
  DW1000.commitConfiguration();
}

void TwrEngine::setAntennaDelay(uint16_t antennaDelay) {
  _antDelay = antennaDelay;
  configure();   // re-tune with the new delay
  startRx();
}

void TwrEngine::startRx() {
  _receivedFlag = false;
  DW1000.newReceive();
  DW1000.setDefaults();
  DW1000.receivePermanently(true);  // chip re-arms RX automatically after a frame
  DW1000.startReceive();
  _radioState = RADIO_ACTIVE;       // every internal re-arm leaves us active
}

// ===========================================================================
// Radio slot control (multi-tag TDMA over the TagLink UART)
// ===========================================================================
void TwrEngine::setRadioState(RadioState state) {
  if (state == _radioState) return;
  switch (state) {
    case RADIO_ACTIVE:
      startRx();
      break;
    case RADIO_IDLE:
      DW1000.idle();          // TRXOFF: no RX events, no TX, chip stays clocked
      break;
    default:
      Serial.printf("[TWR] setRadioState(%u): state reserved, not implemented\n",
                    (unsigned)state);
      return;                 // keep current state
  }
  _radioState = state;
}

// NOTE: DW1000 events are serviced in TASK context (DW1000.pollIrq()) — the pin
// ISR only sets a flag. Every loop that waits on _sentFlag/_receivedFlag must
// call pollIrq(), or the flag will never be set.
bool TwrEngine::waitSent(uint32_t timeoutMs) {
  uint32_t t0 = millis();
  while (!_sentFlag) {
    DW1000.pollIrq();
    if (millis() - t0 > timeoutMs) return false;
    yield();
  }
  _sentFlag = false;
  return true;
}

bool TwrEngine::waitReceived(uint32_t timeoutMs) {
  uint32_t t0 = millis();
  while (!_receivedFlag) {
    DW1000.pollIrq();
    if (millis() - t0 > timeoutMs) return false;
    yield();
  }
  _receivedFlag = false;
  return true;
}

uint16_t TwrEngine::readFrame() {
  uint16_t n = DW1000.getDataLength();
  if (n > UWB_FRAME_MAXLEN) n = UWB_FRAME_MAXLEN;
  DW1000.getData(_rx, n);
  return n;
}

// ===========================================================================
// TAG (initiator)
// ===========================================================================
bool TwrEngine::rangeTo(uint8_t anchorAddr, float& distanceMeters, float& rxPowerDbm) {
  // Watchdog: too many straight failures means the DW1000 receiver is wedged
  // (RXAUTR hardware re-arm has a documented erratum on certain error types).
  // A full configure()+startRx() always recovers it.
  if (_failStreak >= FAIL_STREAK_RESET) {
    Serial.printf("[TWR] radio reset after %u consecutive failures\n", _failStreak);
    _failStreak = 0;
    configure();
    startRx();
  }

  _seq++;

  // 1) POLL (immediate). Record our TX timestamp afterwards.
  DW1000.newTransmit();
  DW1000.setDefaults();
  writeHeader(_tx, MSG_POLL, _myAddr, anchorAddr, _seq);
  DW1000.setData(_tx, UWB_HDR_LEN);
  DW1000.startTransmit();
  if (!waitSent(20)) { startRx(); _failStreak++; return false; }
  DW1000.getTransmitTimestamp(_timePollSent);

  // Listen for POLL_ACK.
  // 45 ms budget: anchor OLED (Adafruit SSD1306 full refresh) blocks ~23 ms at
  // 400kHz I2C or up to ~92 ms at 100kHz if the bus clock was not restored after
  // WiFi re-init. 5 ms reply delay + 45 ms gives comfortable margin for both cases.
  startRx();
  if (!waitReceived(45)) { startRx(); _failStreak++; return false; }
  readFrame();
  if (frameType(_rx) != MSG_POLL_ACK || frameSrc(_rx) != anchorAddr ||
      !frameIsForUs(_rx, _myAddr)) {
    if (frameType(_rx) == MSG_POLL_ACK && frameSrc(_rx) != anchorAddr)
      Serial.printf("[TWR] stray POLL_ACK from 0x%02X while polling 0x%02X"
                    " — check for duplicate anchor IDs\n",
                    frameSrc(_rx), anchorAddr);
    startRx(); _failStreak++; return false;
  }
  DW1000.getReceiveTimestamp(_timePollAckReceived);

  // 2) RANGE (delayed TX so we know our exact send time), carrying our 3 stamps.
  DW1000.newTransmit();
  DW1000.setDefaults();
  writeHeader(_tx, MSG_RANGE, _myAddr, anchorAddr, _seq);
  DW1000Time delay = DW1000Time(UWB_REPLY_DELAY_US, DW1000Time::MICROSECONDS);
  _timeRangeSent = DW1000.setDelay(delay);   // returns the scheduled TX time
  packRangePayload(_tx, _timePollSent, _timePollAckReceived, _timeRangeSent);
  DW1000.setData(_tx, UWB_RANGE_LEN);
  DW1000.startTransmit();
  if (!waitSent(30)) { startRx(); _failStreak++; return false; }

  // Listen for RANGE_REPORT (anchor computed the distance).
  // 30 ms: anchor OLED can block serviceResponder() for ~23 ms after POLL_ACK TX.
  startRx();
  if (!waitReceived(30)) { startRx(); _failStreak++; return false; }
  readFrame();
  if (frameType(_rx) != MSG_RANGE_REPORT || frameSrc(_rx) != anchorAddr) {
    startRx(); _failStreak++; return false;
  }
  // Diagnostics of THIS frame. Force the transceiver idle first: with the
  // auto-re-arm receive mode the receiver is already hunting again by now,
  // and a hunting receiver re-tracks DRX_CARRIER_INT - reading it live
  // returns ~0 (same mechanism as the 2026-07-21 all-zero CIR captures).
  // The exchange is complete at this point, so a brief idle is safe;
  // captureCir has used the identical pattern since 1c10187.
  DW1000.idle();
  unpackReportPayload(_rx, distanceMeters, rxPowerDbm);
  // Phase 2.1 NLOS: read total RX power AND first-path power of the SAME received
  // frame (this RANGE_REPORT) so the host's NLOS score (rx - fp) is a valid
  // same-frame comparison. This overrides the anchor-reported rx (which was
  // measured on the other link, making rx-fp physically meaningless).
  rxPowerDbm = DW1000.getReceivePower();
  _fpPower   = DW1000.getFirstPathPower();
  _quality   = DW1000.getReceiveQuality();
  _carrierInt = DW1000.getCarrierIntegrator();
  _failStreak = 0;
  startRx();
  return true;
}

// Phase-C diagnostics: DW1000 die temperature + Vbat via the SAR ADC. The
// manual's 6.4 sequence pokes RF_CONF/TX_CAL, so the transceiver must be
// idle (same lesson as the CIR capture: never touch analog state with the
// receiver hunting). Cost is a handful of SPI ops - fine once per sweep.
void TwrEngine::readTempVbat(float& tempC, float& vbat) {
  DW1000.idle();
  DW1000.getTempAndVbat(tempC, vbat);
  startRx();
}

// ===========================================================================
// TAG: CIR capture (diagnostics)
// ===========================================================================
// Deliberately DUPLICATES the proven rangeTo() flow instead of refactoring
// it: the accumulator must be read after the RANGE_REPORT arrives and
// BEFORE the receiver is re-armed (any newly received frame overwrites it).
bool TwrEngine::captureCir(uint8_t anchorAddr, float& distanceMeters,
                           float& rxPowerDbm, uint16_t& fpIndexRaw,
                           byte* cirBuf, uint16_t nTaps) {
  _seq++;

  DW1000.newTransmit();
  DW1000.setDefaults();
  writeHeader(_tx, MSG_POLL, _myAddr, anchorAddr, _seq);
  DW1000.setData(_tx, UWB_HDR_LEN);
  DW1000.startTransmit();
  if (!waitSent(20)) { startRx(); return false; }
  DW1000.getTransmitTimestamp(_timePollSent);

  startRx();
  if (!waitReceived(45)) { startRx(); return false; }
  readFrame();
  if (frameType(_rx) != MSG_POLL_ACK || frameSrc(_rx) != anchorAddr ||
      !frameIsForUs(_rx, _myAddr)) {
    startRx(); return false;
  }
  DW1000.getReceiveTimestamp(_timePollAckReceived);

  DW1000.newTransmit();
  DW1000.setDefaults();
  writeHeader(_tx, MSG_RANGE, _myAddr, anchorAddr, _seq);
  DW1000Time delay = DW1000Time(UWB_REPLY_DELAY_US, DW1000Time::MICROSECONDS);
  _timeRangeSent = DW1000.setDelay(delay);
  packRangePayload(_tx, _timePollSent, _timePollAckReceived, _timeRangeSent);
  DW1000.setData(_tx, UWB_RANGE_LEN);
  DW1000.startTransmit();
  if (!waitSent(30)) { startRx(); return false; }

  startRx();
  if (!waitReceived(30)) { startRx(); return false; }
  readFrame();
  if (frameType(_rx) != MSG_RANGE_REPORT || frameSrc(_rx) != anchorAddr) {
    startRx(); return false;
  }
  // Force the transceiver OFF before touching the accumulator: with the
  // auto-re-arm receive mode the receiver is already hunting for the next
  // preamble by now, and an ACTIVE receiver clears/rewrites ACC_MEM -
  // reading it live returns zeros (hence the 2026-07-21 all-zero captures).
  DW1000.idle();
  unpackReportPayload(_rx, distanceMeters, rxPowerDbm);
  rxPowerDbm = DW1000.getReceivePower();
  _fpPower   = DW1000.getFirstPathPower();
  _quality   = DW1000.getReceiveQuality();
  fpIndexRaw = DW1000.getFirstPathIndex();
  DW1000.readAccumulator(cirBuf, (uint16_t)(nTaps * 4));
  startRx();
  return true;
}

// ===========================================================================
// TAG (survey initiator) — ask an anchor to range to another anchor
// ===========================================================================
bool TwrEngine::surveyRequest(uint8_t anchorAddr, uint8_t targetAddr,
                               float& distanceMeters, float& rxPowerDbm) {
  _seq++;

  // Send SURVEY_REQ to anchorAddr telling it to range to targetAddr.
  DW1000.newTransmit();
  DW1000.setDefaults();
  writeHeader(_tx, MSG_SURVEY_REQ, _myAddr, anchorAddr, _seq);
  packSurveyReq(_tx, targetAddr);
  DW1000.setData(_tx, UWB_SURVEY_REQ_LEN);
  DW1000.startTransmit();
  if (!waitSent(20)) { startRx(); return false; }

  // Wait for the anchor to complete its own TWR exchange and reply.
  // Budget: reply_delay(5ms) + TWR frames(~20ms) + resp TX(~5ms) + margin.
  //
  // IMPORTANT: we OVERHEAR every frame of the anchor's own exchange with the
  // target (POLL/POLL_ACK/RANGE/RANGE_REPORT are on the same channel and RX
  // is not destination-filtered in hardware). Keep listening until the
  // SURVEY_RESP addressed to us arrives; a single waitReceived() here would
  // give up on the first overheard POLL and fail ~every request.
  const uint32_t budgetMs = 200;
  uint32_t t0 = millis();
  startRx();
  for (;;) {
    uint32_t elapsed = millis() - t0;
    if (elapsed >= budgetMs) { startRx(); return false; }
    if (!waitReceived(budgetMs - elapsed)) { startRx(); return false; }
    readFrame();
    if (frameType(_rx) == MSG_SURVEY_RESP && frameSrc(_rx) == anchorAddr &&
        frameIsForUs(_rx, _myAddr)) {
      break;                      // our reply
    }
    startRx();                    // overheard survey-exchange frame — keep waiting
  }

  uint8_t target;
  unpackSurveyResp(_rx, target, distanceMeters, rxPowerDbm);
  startRx();
  // Zero distance means the anchor's ranging attempt failed.
  return (target == targetAddr) && (distanceMeters > 0.0f);
}

// ===========================================================================
// TAG (antenna delay push) — send new antenna delay to an anchor, wait for ACK
// ===========================================================================
bool TwrEngine::pushAntDelay(uint8_t anchorAddr, uint16_t delayTicks, uint8_t maxRetries) {
  for (uint8_t attempt = 0; attempt < maxRetries; attempt++) {
    _seq++;

    DW1000.newTransmit();
    DW1000.setDefaults();
    writeHeader(_tx, MSG_ANT_DELAY, _myAddr, anchorAddr, _seq);
    packAntDelay(_tx, delayTicks);
    DW1000.setData(_tx, UWB_ANT_DELAY_LEN);
    DW1000.startTransmit();
    if (!waitSent(20)) { startRx(); continue; }

    startRx();
    if (!waitReceived(2000)) { startRx(); continue; }  // 2 s — anchor needs time to reconfigure
    readFrame();

    if (frameType(_rx) == MSG_ANT_DELAY_ACK && frameSrc(_rx) == anchorAddr &&
        frameIsForUs(_rx, _myAddr)) {
      startRx();
      return true;
    }
    startRx();
  }
  return false;
}

// ===========================================================================
// TAG (multi-tag token ring) — broadcast token / announce, observe frames
// ===========================================================================
bool TwrEngine::sendSlotGrant(uint8_t grantedAddr) {
  _seq++;
  DW1000.newTransmit();
  DW1000.setDefaults();
  writeHeader(_tx, MSG_SLOT_GRANT, _myAddr, UWB_ADDR_BROADCAST, _seq);
  packSlotGrant(_tx, grantedAddr);
  DW1000.setData(_tx, UWB_SLOT_GRANT_LEN);
  DW1000.startTransmit();
  bool ok = waitSent(20);
  startRx();
  return ok;
}

bool TwrEngine::sendAnnounce(uint8_t tagAddr) {
  _seq++;
  DW1000.newTransmit();
  DW1000.setDefaults();
  writeHeader(_tx, MSG_ANNOUNCE, _myAddr, UWB_ADDR_BROADCAST, _seq);
  packAnnounce(_tx, tagAddr);
  DW1000.setData(_tx, UWB_ANNOUNCE_LEN);
  DW1000.startTransmit();
  bool ok = waitSent(20);
  startRx();
  return ok;
}

bool TwrEngine::pollFrame(uint8_t& type, uint8_t& src, uint8_t& payload0) {
  DW1000.pollIrq();                     // service DW1000 events (task context)
  if (!_receivedFlag) return false;
  _receivedFlag = false;
  _lastRxMs = millis();                 // keep the silence watchdog fed
  uint16_t n = readFrame();
  type     = frameType(_rx);
  src      = frameSrc(_rx);
  payload0 = (n > UWB_HDR_LEN) ? _rx[UWB_HDR_LEN] : UWB_ADDR_INVALID;
  startRx();                            // re-arm immediately
  return true;
}

// ===========================================================================
// ANCHOR (responder)
// ===========================================================================
bool TwrEngine::serviceResponder() {
  DW1000.pollIrq();                     // service DW1000 events (task context)
  // Watchdog: if the receiver has been silent too long, the DW1000 is wedged.
  // Reset it so the anchor doesn't go permanently deaf.
  uint32_t now = millis();
  if (_lastRxMs == 0) _lastRxMs = now;   // initialise on first call
  if (now - _lastRxMs > ANCHOR_RX_WATCHDOG_MS) {
    Serial.printf("[TWR] anchor RX watchdog fired (%lu ms silent) - resetting\n",
                  now - _lastRxMs);
    _lastRxMs = now;
    configure();
    startRx();
  }

  if (!_receivedFlag) return false;
  _receivedFlag = false;
  _lastRxMs = millis();

  readFrame();
  if (!frameIsForUs(_rx, _myAddr)) { startRx(); return false; }

  const uint8_t type = frameType(_rx);
  const uint8_t src  = frameSrc(_rx);
  const uint8_t seq  = frameSeq(_rx);

  if (type == MSG_POLL) {
    DW1000.getReceiveTimestamp(_timePollReceived);
    _peer = src;

    // Reply POLL_ACK (delayed), then capture our actual TX time.
    DW1000.newTransmit();
    DW1000.setDefaults();
    writeHeader(_tx, MSG_POLL_ACK, _myAddr, src, seq);
    DW1000Time delay = DW1000Time(UWB_REPLY_DELAY_US, DW1000Time::MICROSECONDS);
    DW1000.setDelay(delay);
    DW1000.setData(_tx, UWB_HDR_LEN);
    DW1000.startTransmit();
    if (waitSent(30)) DW1000.getTransmitTimestamp(_timePollAckSent);
    startRx();
    return false;   // first half only; RANGE still pending

  } else if (type == MSG_RANGE && src == _peer) {
    DW1000.getReceiveTimestamp(_timeRangeReceived);

    DW1000Time tPollSent, tPollAckReceived, tRangeSent;
    unpackRangePayload(_rx, tPollSent, tPollAckReceived, tRangeSent);

    // Asymmetric double-sided TWR (cancels clock-frequency offset).
    DW1000Time round1 = (tPollAckReceived   - tPollSent).wrap();
    DW1000Time reply1 = (_timePollAckSent    - _timePollReceived).wrap();
    DW1000Time round2 = (_timeRangeReceived  - _timePollAckSent).wrap();
    DW1000Time reply2 = (tRangeSent          - tPollAckReceived).wrap();

    DW1000Time tof;
    tof.setTimestamp((round1 * round2 - reply1 * reply2) /
                     (round1 + round2 + reply1 + reply2));
    float dist = tof.getAsMeters();
    float rxp  = DW1000.getReceivePower();

    // Reply RANGE_REPORT (immediate) with the computed distance.
    DW1000.newTransmit();
    DW1000.setDefaults();
    writeHeader(_tx, MSG_RANGE_REPORT, _myAddr, src, seq);
    packReportPayload(_tx, dist, rxp);
    DW1000.setData(_tx, UWB_REPORT_LEN);
    DW1000.startTransmit();
    waitSent(30);

    _lastDistance = dist;
    _lastPeer     = src;
    startRx();
    return true;    // complete TWR exchange — safe window for OLED update

  } else if (type == MSG_SURVEY_REQ) {
    // Anchor temporarily acts as initiator to range to the requested target.
    uint8_t target = unpackSurveyReqTarget(_rx);
    float dist = 0.0f, rxp = 0.0f;
    bool ok = rangeTo(target, dist, rxp);  // uses tag-side path; radio re-armed on return

    // Reply with result regardless of success so the tag's timeout doesn't fire.
    DW1000.newTransmit();
    DW1000.setDefaults();
    writeHeader(_tx, MSG_SURVEY_RESP, _myAddr, src, seq);
    packSurveyResp(_tx, target, ok ? dist : 0.0f, ok ? rxp : 0.0f);
    DW1000.setData(_tx, UWB_SURVEY_RESP_LEN);
    DW1000.startTransmit();
    waitSent(30);
    startRx();
    return false;

  } else if (type == MSG_ANT_DELAY) {
    uint16_t newDelay = unpackAntDelay(_rx);
    setAntennaDelay(newDelay);   // applies configure() + startRx()
    _antDelayUpdated = true;
    Serial.printf("[CAL] Antenna delay updated: %u (from 0x%02X)\n", newDelay, src);

    // ACK so the tag (and host) can confirm delivery.
    DW1000.newTransmit();
    DW1000.setDefaults();
    writeHeader(_tx, MSG_ANT_DELAY_ACK, _myAddr, src, seq);
    packAntDelay(_tx, newDelay);
    DW1000.setData(_tx, UWB_ANT_DELAY_LEN);
    DW1000.startTransmit();
    waitSent(30);
    startRx();
    return false;

  } else {
    startRx();
    return false;
  }
}

void TwrEngine::printDeviceId() {
  char msg[128];
  DW1000.getPrintableDeviceIdentifier(msg);
  Serial.print(F("DW1000 device id: "));
  Serial.println(msg);
}
