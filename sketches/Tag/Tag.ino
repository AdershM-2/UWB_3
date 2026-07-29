/*
 * Tag.ino  –  UWB tag WITHOUT IMU, for the same ESP32-WROVER + DW1000 hardware
 *             as TagWrover (this fleet's two tag boards are identical; only one
 *             carries a BNO085).
 *
 * This is a direct port of TagWrover.ino with all BNO085 code removed and the
 * per-board identity set to tag 2 (0xF1). Keep the two sketches' setup()/loop()
 * structure in lockstep when editing either.
 *
 * Peripheral map (same as TagWrover)
 * ──────────────────────────────────────────────────────────────────────
 *  DW1000 (SPI) : SCK=18  MISO=19  MOSI=23  CS=4   RST=27  IRQ=34
 *
 *  No OLED and no IMU on this board (removed 2026-07-26 — this is a bench
 *  slave, the display/IMU cost was never earning its keep here; GPIO 32/33
 *  are free). ⚠ GPIO 16/17 reserved for PSRAM on WROVER.
 *
 * The previous Makerfabs "ESP32 UWB Pro with Display" version of this sketch
 * (default pin map, SensorImu stub) is preserved as Tag.ino.pro-board.bak.
 */

// ── Transport: pick exactly ONE ───────────────────────────────────────────────
#define UWB_HOSTLINK_SERIAL
//#define UWB_HOSTLINK_UDP
// UWB_USE_OLED intentionally NOT defined — no display on this board (removed
// 2026-07-26). OledStatus is a no-op stub in this state; nothing else here
// depends on it, so leaving it undefined is sufficient — see also the object/
// call-site removal below, done for real flash/RAM savings, not just silence.

// ── Coordination: pick exactly ONE ───────────────────────────────────────────
// RING (default) — the two tags pass a token OVER THE AIR and each streams to
//   the host independently. This is the long-proven WiFi setup.
// WIRE — this board becomes SLAVE: no WiFi, no host link. Its radio stays
//   parked (TRXOFF) except in the slot the master grants it over UART2, and its
//   sweep goes back to the master on the same cable, which forwards it to the
//   host. USB serial stays live for bench work (SURVEY, calibration, debug).
//
// To switch this board to wired slave mode, swap the comment markers on the
// next two lines. Leave the transport on UDP or SERIAL as you like — in WIRE
// mode this board's host link is unused either way, so SERIAL is the honest
// choice and avoids associating with WiFi at all.
//#define UWB_COORD_RING
 #define UWB_COORD_WIRE
//
// Wiring for WIRE mode (a shared ground is REQUIRED, not optional):
//   this board GPIO 22 (TX) ──> master GPIO 35 (RX)
//   this board GPIO 35 (RX) <── master GPIO 22 (TX)
//   GND <──> GND
#if defined(UWB_COORD_WIRE) && defined(UWB_WIRE_ROLE_MASTER)
#error "Tag.ino is the non-IMU board: it is the SLAVE, do not define UWB_WIRE_ROLE_MASTER"
#endif

// ── Board pin overrides — MUST appear before #include <UwbRtls.h> ────────────
// ESP32 UWB (non-Pro) uses DW1000 CS=4, not 21.
#define UWB_PIN_SS    4    // DW1000 chip select (hardware-wired on this board)

#include <UwbRtls.h>
#include <Wire.h>
#include <Preferences.h>
#include <esp_timer.h>   // monotonic µs clock for the TagLink slot timing

// ── Configuration — edit these for your setup ─────────────────────────────────
static const uint8_t  TAG_ID        = 0xF1;  // 0xF1 — the TagWrover board is 0xF0
// Factory default only — an NVS-stored value (from SETMYDELAY / HWCALIB_SELF)
// overrides this at boot, so delay retuning never needs a reflash.
static const uint16_t ANTENNA_DELAY = 16473; // Tag 2 (0xF1) calibrated (3.0 m ref, 2026-06-18)

static const uint8_t ANCHORS[]  = { 0x01, 0x02, 0x03, 0x04, 0x05 };
static const uint8_t N_ANCHORS  = sizeof(ANCHORS) / sizeof(ANCHORS[0]);

// Multi-tag token ring: ordered list of ALL tag addresses sharing the channel.
// MUST be identical on every tag board (default from UwbConfig.h).
static const uint8_t TAG_RING[] = UWB_TAG_RING_INIT;
static const uint8_t RING_SIZE  = sizeof(TAG_RING) / sizeof(TAG_RING[0]);

#if defined(UWB_HOSTLINK_UDP)
static const char*    WIFI_SSID = "iitk";          // ← fill in
static const char*    WIFI_PASS = "";                    // ← "" for open network
static IPAddress      HOST_IP(255, 255, 255, 255);       // ← Python/MATLAB PC IP
static const uint16_t HOST_PORT  = 4100;
#endif

// Survey averaging samples (100 per anchor pair)
static const uint16_t SURVEY_SAMPLES = 100;

// Hardware antenna-delay calibration (GUI-triggered binary search per anchor).
static const uint8_t  CALIB_ITERS    = 14;
static const uint16_t CALIB_SAMPLES  = 50;
static const uint16_t CALIB_DELAY_LO = 15800;
static const uint16_t CALIB_DELAY_HI = 16900;

// ── Objects ───────────────────────────────────────────────────────────────────
TwrEngine    engine;
UwbScheduler scheduler;
HostLink     host;
Preferences  prefs;

// Coordination — one of the two, chosen by the #define block at the top.
#if defined(UWB_COORD_WIRE)
TagLink      tagLink;   // wired master/slave TDMA over UART2 (this board = SLAVE)
#else
TagRing      ring;      // over-air token (one tag transmits at a time)
#endif

static inline void coordBegin() {
#if defined(UWB_COORD_WIRE)
  tagLink.begin(TAG_ROLE_SLAVE, TAG_ID, /*caps=*/0);   // no IMU on this board
  engine.setRadioState(RADIO_IDLE);   // park the radio until the master grants
#else
  ring.begin(&engine, TAG_ID, TAG_RING, RING_SIZE);
#endif
}

static uint16_t g_antDelay = ANTENNA_DELAY;  // active own delay (NVS-backed)

// ── Anchor self-survey (same as TagWrover) ────────────────────────────────────
static void runSurvey() {
  uint8_t pairs = (N_ANCHORS * (N_ANCHORS - 1)) / 2;
  char buf[64];

  snprintf(buf, sizeof(buf), "SURVEY_BEGIN,v1,%u\n", pairs);
  host.sendRaw(buf);
  Serial.printf("Survey: %u anchors, %u pairs, %u samples each\n",
                N_ANCHORS, pairs, SURVEY_SAMPLES);

  uint8_t pairIdx = 0;
  for (uint8_t i = 0; i < N_ANCHORS; i++) {
    for (uint8_t j = i + 1; j < N_ANCHORS; j++) {
      uint8_t a = ANCHORS[i], b = ANCHORS[j];
      float   sum = 0.0f;
      uint16_t ok = 0;
      pairIdx++;

      Serial.printf("  Pair %u/%u  0x%02X -> 0x%02X:\n", pairIdx, pairs, a, b);

      for (uint16_t s = 0; s < SURVEY_SAMPLES; s++) {
        float dist, rxp;
        if (engine.surveyRequest(a, b, dist, rxp)) { sum += dist; ok++; }
        if ((s & 0xF) == 0xF) {
          Serial.printf("    [%u/%u ok=%u]\n", s + 1, SURVEY_SAMPLES, ok);
        }
        delay(5);
      }

      if (ok >= SURVEY_SAMPLES / 4) {
        uint32_t mm = (uint32_t)lroundf(sum / ok * 1000.0f);
        snprintf(buf, sizeof(buf), "SURVEY,v1,%u,%u,%lu,%u\n", a, b, mm, ok);
        host.sendRaw(buf);
        Serial.printf("  => %.3f m  (%u/%u ok)\n", sum / ok, ok, SURVEY_SAMPLES);
      } else {
        Serial.printf("  FAILED (ok=%u/%u) — check link 0x%02X<->0x%02X\n",
                      ok, SURVEY_SAMPLES, a, b);
      }
    }
  }
  host.sendRaw("SURVEY_DONE,v1\n");
  Serial.println("Survey complete.");
}

static char g_bootMsg[40];   // reset forensics line, sent once the host link is up

// ── setup ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(300);
  ResetDiag::report(g_bootMsg, sizeof(g_bootMsg));

  ChargeMode::begin();
  ChargeMode::checkAndSleep();   // short GPIO13-14 -> charge mode (deep sleep)

  // Own antenna delay: NVS-calibrated value overrides the sketch constant
  // (SETMYDELAY / HWCALIB_SELF persist here — retuning needs no reflash).
  prefs.begin("uwb", false);
  g_antDelay = prefs.getUShort("tagDelay", ANTENNA_DELAY);
  Serial.printf("[CAL] Tag antenna delay: %u%s\n", g_antDelay,
                g_antDelay == ANTENNA_DELAY ? " (sketch default)" : " (from NVS)");

  // ── WiFi / transport init ────────────────────────────────────────────────
#if defined(UWB_HOSTLINK_UDP)
  host.begin(WIFI_SSID, WIFI_PASS, HOST_IP, HOST_PORT);
  // ── UWB radio LAST — after WiFi settled ────────────────────────────────
  // Explicit SPI.begin() because the WiFi bus teardown above can leave the
  // ESP32 SPI peripheral in an undefined state on WROVER modules.
  // "DECA 01302001" in Serial → CS=4 correct; all zeros → wrong CS pin.
  SPI.begin(UWB_PIN_SCK, UWB_PIN_MISO, UWB_PIN_MOSI);
  engine.begin(TWR_TAG, TAG_ID, g_antDelay);
  scheduler.begin(&engine, ANCHORS, N_ANCHORS);
  coordBegin();
  engine.printDeviceId();
#else
  host.begin(115200);
  SPI.begin(UWB_PIN_SCK, UWB_PIN_MISO, UWB_PIN_MOSI);
  engine.begin(TWR_TAG, TAG_ID, g_antDelay);
  scheduler.begin(&engine, ANCHORS, N_ANCHORS);
  coordBegin();
  engine.printDeviceId();
#endif
  host.sendRaw(g_bootMsg);   // why did we last reset? (PANIC/WDT/BROWNOUT/...)

  Serial.printf("Tag 0x%02X ready — CS=%d  %u anchors  IMU:NO\n",
                TAG_ID, UWB_PIN_SS, N_ANCHORS);
}

// ── Antenna delay push relay (same as TagWrover) ──────────────────────────────
static void handleSetAntDelay(const char* cmd) {
  unsigned int aid = 0, ticks = 0;
  if (sscanf(cmd, "SETANTDELAY,%u,%u", &aid, &ticks) != 2) {
    Serial.println("[CAL] SETANTDELAY parse error — expected SETANTDELAY,<id>,<ticks>");
    return;
  }
  uint8_t  anchorAddr = (uint8_t)aid;
  uint16_t delayTicks = (uint16_t)ticks;
  Serial.printf("[CAL] Pushing delay %u to anchor 0x%02X ...\n", delayTicks, anchorAddr);
  bool ok = engine.pushAntDelay(anchorAddr, delayTicks);
  char buf[48];
  snprintf(buf, sizeof(buf), ok ? "ANTDELAY_ACK,%u,%u\n" : "ANTDELAY_FAIL,%u,%u\n",
           anchorAddr, delayTicks);
  host.sendRaw(buf);
  Serial.printf("[CAL] %s 0x%02X\n", ok ? "ACK from" : "No ACK from", anchorAddr);
}

static void handleHwCalib(const char* cmd) {
  unsigned int aid, trueDistMm;
  if (sscanf(cmd, "HWCALIB,%u,%u", &aid, &trueDistMm) != 2) {
    host.sendRaw("HWCALIB_FAIL,0,parse_error\n");
    return;
  }
  uint8_t anchorAddr = (uint8_t)aid;
  Serial.printf("[HWCAL] Calibrating anchor 0x%02X at true dist %u mm\n",
                anchorAddr, trueDistMm);

  uint16_t low = CALIB_DELAY_LO, high = CALIB_DELAY_HI;
  for (uint8_t it = 0; it < CALIB_ITERS; it++) {
    uint16_t mid = (low + high) / 2;
    if (!engine.pushAntDelay(anchorAddr, mid, 1)) {
      char buf[48];
      snprintf(buf, sizeof(buf), "HWCALIB_FAIL,%u,push_timeout\n", aid);
      host.sendRaw(buf);
      return;
    }
    float sum = 0.0f; uint16_t ok = 0;
    for (uint16_t s = 0; s < CALIB_SAMPLES; s++) {
      float dist, rxp;
      if (engine.rangeTo(anchorAddr, dist, rxp)) { sum += dist; ok++; }
      delay(5);
    }
    if (ok < CALIB_SAMPLES / 4) {
      char buf[48];
      snprintf(buf, sizeof(buf), "HWCALIB_FAIL,%u,no_range\n", aid);
      host.sendRaw(buf);
      return;
    }
    float meanMm = (sum / ok) * 1000.0f;
    int32_t errMm = (int32_t)(meanMm - (float)trueDistMm);
    char buf[64];
    snprintf(buf, sizeof(buf), "HWCALIB_PROG,%u,%u,%u,%u,%ld\n",
             aid, it, mid, (uint32_t)meanMm, (long)errMm);
    host.sendRaw(buf);
    Serial.printf("  [%2u/%u] delay=%u  mean=%u mm  err=%+ld mm\n",
                  (unsigned)(it + 1), (unsigned)CALIB_ITERS,
                  mid, (uint32_t)meanMm, (long)errMm);
    if (meanMm > (float)trueDistMm) low = mid; else high = mid;
  }
  uint16_t finalDelay = (low + high) / 2;
  bool pushOk = engine.pushAntDelay(anchorAddr, finalDelay, 3);
  char buf[48];
  snprintf(buf, sizeof(buf), pushOk ? "HWCALIB_DONE,%u,%u\n"
                                    : "HWCALIB_FAIL,%u,final_push\n",
           aid, finalDelay);
  host.sendRaw(buf);
  Serial.printf("[HWCAL] Anchor 0x%02X: %s  delay=%u\n",
                anchorAddr, pushOk ? "DONE" : "FAIL", finalDelay);
}

// ── Own antenna-delay control (tag-side twin of the anchor NVS flow) ─────────
static void applyMyDelay(uint16_t ticks) {
  engine.setAntennaDelay(ticks);    // applies configure() + startRx()
  g_antDelay = ticks;
  prefs.putUShort("tagDelay", ticks);
}

// All own-delay commands carry the target TAG_ID as the first field so a UDP
// broadcast is safe with several tags powered — non-addressees stay silent.
static void handleSetMyDelay(const char* cmd) {
  unsigned int tid = 0, ticks = 0;
  if (sscanf(cmd, "SETMYDELAY,%u,%u", &tid, &ticks) != 2) {
    host.sendRaw("MYDELAY_FAIL,parse_error\n");
    return;
  }
  if ((uint8_t)tid != TAG_ID) return;   // addressed to another tag
  applyMyDelay((uint16_t)ticks);
  char buf[48];
  snprintf(buf, sizeof(buf), "MYDELAY_ACK,%u,%u\n", TAG_ID, ticks);
  host.sendRaw(buf);
  Serial.printf("[CAL] Own antenna delay set: %u (NVS saved)\n", ticks);
}

static void handleGetMyDelay(const char* cmd) {
  unsigned int tid = 0;
  if (sscanf(cmd, "GETMYDELAY,%u", &tid) != 1) return;
  if ((uint8_t)tid != TAG_ID && tid != 0) return;   // 0 = query all tags
  char buf[48];
  snprintf(buf, sizeof(buf), "MYDELAY_ACK,%u,%u\n", TAG_ID, g_antDelay);
  host.sendRaw(buf);
}

// Binary-search OUR OWN antenna delay against one already-calibrated anchor at
// a tape-measured distance. Mirrors handleHwCalib but adjusts this tag instead
// of pushing to the anchor — per-tag bias must not be absorbed into anchor NVS
// (shared anchors serve every tag in the fleet).
static void handleHwCalibSelf(const char* cmd) {
  unsigned int tid, aid, trueDistMm;
  if (sscanf(cmd, "HWCALIB_SELF,%u,%u,%u", &tid, &aid, &trueDistMm) != 3) {
    host.sendRaw("HWCALIB_SELF_FAIL,0,parse_error\n");
    return;
  }
  if ((uint8_t)tid != TAG_ID) return;   // addressed to another tag
  uint8_t anchorAddr = (uint8_t)aid;
  Serial.printf("[HWCAL-SELF] Calibrating own delay vs anchor 0x%02X at %u mm\n",
                anchorAddr, trueDistMm);

  uint16_t low = CALIB_DELAY_LO, high = CALIB_DELAY_HI;
  for (uint8_t it = 0; it < CALIB_ITERS; it++) {
    uint16_t mid = (low + high) / 2;
    engine.setAntennaDelay(mid);              // adjust OURSELVES, not the anchor
    float sum = 0.0f; uint16_t ok = 0;
    for (uint16_t s = 0; s < CALIB_SAMPLES; s++) {
      float dist, rxp;
      if (engine.rangeTo(anchorAddr, dist, rxp)) { sum += dist; ok++; }
      delay(5);
    }
    if (ok < CALIB_SAMPLES / 4) {
      engine.setAntennaDelay(g_antDelay);     // restore last good value
      char buf[48];
      snprintf(buf, sizeof(buf), "HWCALIB_SELF_FAIL,%u,no_range\n", aid);
      host.sendRaw(buf);
      return;
    }
    float meanMm = (sum / ok) * 1000.0f;
    int32_t errMm = (int32_t)(meanMm - (float)trueDistMm);
    char buf[64];
    snprintf(buf, sizeof(buf), "HWCALIB_SELF_PROG,%u,%u,%u,%u,%ld\n",
             aid, it, mid, (uint32_t)meanMm, (long)errMm);
    host.sendRaw(buf);
    Serial.printf("  [%2u/%u] delay=%u  mean=%u mm  err=%+ld mm\n",
                  (unsigned)(it + 1), (unsigned)CALIB_ITERS,
                  mid, (uint32_t)meanMm, (long)errMm);
    if (meanMm > (float)trueDistMm) low = mid; else high = mid;
  }
  uint16_t finalDelay = (low + high) / 2;
  applyMyDelay(finalDelay);                    // apply + persist to NVS
  char buf[48];
  snprintf(buf, sizeof(buf), "HWCALIB_SELF_DONE,%u,%u\n", TAG_ID, finalDelay);
  host.sendRaw(buf);
  Serial.printf("[HWCAL-SELF] DONE  own delay=%u (NVS saved)\n", finalDelay);
}

static void dispatchCmd(const char* cmd) {
  if (strcmp(cmd, "SURVEY") == 0)
    runSurvey();
  else if (strncmp(cmd, "SETANTDELAY,", 12) == 0)
    handleSetAntDelay(cmd);
  else if (strncmp(cmd, "HWCALIB_SELF,", 13) == 0)
    handleHwCalibSelf(cmd);
  else if (strncmp(cmd, "HWCALIB,", 8) == 0)
    handleHwCalib(cmd);
  else if (strncmp(cmd, "SETMYDELAY,", 11) == 0)
    handleSetMyDelay(cmd);
  else if (strncmp(cmd, "GETMYDELAY,", 11) == 0)
    handleGetMyDelay(cmd);
}

// ── loop ──────────────────────────────────────────────────────────────────────
// Both cycle bodies are defined below; declare them so loop() can sit first.
#if defined(UWB_COORD_WIRE)
static void slaveCycle();
#else
static void ringCycle();
#endif

void loop() {
  ChargeMode::checkAndSleep();   // jumper 13-14 enters charge mode at any time

  // ── UDP command check (calibration tool sends SETANTDELAY here) ───────────
  static char udpCmd[32];
  if (host.receiveCmd(udpCmd, sizeof(udpCmd)) > 0)
    dispatchCmd(udpCmd);

  // ── Serial command check (SURVEY trigger, debug) ──────────────────────────
  static char    cmdBuf[32];
  static uint8_t cmdLen = 0;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      cmdBuf[cmdLen] = '\0';
      dispatchCmd(cmdBuf);
      cmdLen = 0;
    } else if (cmdLen < sizeof(cmdBuf) - 1) {
      cmdBuf[cmdLen++] = c;
    }
  }

#if defined(UWB_COORD_WIRE)
  tagLink.poll();
  slaveCycle();
#else
  ringCycle();
#endif
}

// ── RING cycle (over-air token) ───────────────────────────────────────────────
#if !defined(UWB_COORD_WIRE)
static void ringCycle() {
  if (ring.poll()) {
    scheduler.sweep();
    host.sendSweep(millis(), TAG_ID, scheduler, nullptr);  // no IMU on this board
    ring.handoff();
  }
}
#endif

// ── SLAVE cycle (TagLink) ─────────────────────────────────────────────────────
// Radio parked until the master grants a slot; the sweep goes back up the wire
// as binary records, and the master rebuilds the RTLS line with the same
// serializer it uses for its own sweep.
#if defined(UWB_COORD_WIRE)
static void slaveCycle() {
  uint16_t cycle, budgetMs;
  if (!tagLink.pollGo(cycle, budgetMs)) return;

  engine.setRadioState(RADIO_ACTIVE);
  int64_t t0 = esp_timer_get_time();
  scheduler.sweep();
  uint32_t sweepUs = (uint32_t)(esp_timer_get_time() - t0);
  engine.setRadioState(RADIO_IDLE);

  RangeResult results[UWB_MAX_ANCHORS];
  uint8_t n = scheduler.anchorCount();
  if (n > UWB_MAX_ANCHORS) n = UWB_MAX_ANCHORS;
  for (uint8_t i = 0; i < n; i++) results[i] = scheduler.result(i);

  tagLink.sendData(millis(), TAG_ID, sweepUs, results, n);
  tagLink.sendDone();

  // Serial mirror for bench debugging (same serializer the master uses). This
  // is what makes an idle-but-healthy slave distinguishable from a dead one.
  char line[768];
  int len = HostLink::format(line, sizeof(line), millis(), TAG_ID, results, n);
  if (len > 0) Serial.write(line, len);
}
#endif
