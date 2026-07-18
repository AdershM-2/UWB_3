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
 *  SSD1306 OLED : SDA=32  SCL=33   (Wire, I2C address 0x3C or 0x3D)
 *
 *  GPIO 4 = DW1000 CS on this board, so the default OLED SDA=4 conflicts.
 *  OLED moved to 32/33 (Wire). ⚠ GPIO 16/17 reserved for PSRAM on WROVER.
 *
 * The previous Makerfabs "ESP32 UWB Pro with Display" version of this sketch
 * (default pin map, SensorImu stub) is preserved as Tag.ino.pro-board.bak.
 */

// ── Transport: pick exactly ONE ───────────────────────────────────────────────
// #define UWB_HOSTLINK_SERIAL
#define UWB_HOSTLINK_UDP
#define UWB_USE_OLED

// ── Board pin overrides — MUST appear before #include <UwbRtls.h> ────────────
// ESP32 UWB (non-Pro) uses DW1000 CS=4, not 21.
// GPIO 4 is also the default OLED SDA in UwbConfig.h — conflict!
// Fix: keep DW1000 CS=4 (hardware-wired) and move OLED to GPIO 32/33.
#define UWB_PIN_SS    4    // DW1000 chip select (hardware-wired on this board)
#define OLED_PIN_SDA 32    // OLED I2C SDA — moved off GPIO 4 to avoid CS clash
#define OLED_PIN_SCL 33    // OLED I2C SCL

#include <UwbRtls.h>
#include <Wire.h>
#include <Preferences.h>

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
TagRing      ring;    // multi-tag coordination (one tag transmits at a time)
HostLink     host;
OledStatus   oled;
Preferences  prefs;

static uint16_t g_antDelay = ANTENNA_DELAY;  // active own delay (NVS-backed)

// Update OLED: UWB status (no IMU on this board).
static void updateOled(uint8_t nGood) {
  char l0[22], l1[22];
  snprintf(l0, 22, "TAG:%02X  %u/%u OK", TAG_ID, nGood, N_ANCHORS);
  snprintf(l1, 22, "Delay: %u", g_antDelay);
  oled.show(l0, l1, "", "");
}

// ── Anchor self-survey (same as TagWrover) ────────────────────────────────────
static void runSurvey() {
  uint8_t pairs = (N_ANCHORS * (N_ANCHORS - 1)) / 2;
  char buf[64];

  snprintf(buf, sizeof(buf), "SURVEY_BEGIN,v1,%u\n", pairs);
  host.sendRaw(buf);
  Serial.printf("Survey: %u anchors, %u pairs, %u samples each\n",
                N_ANCHORS, pairs, SURVEY_SAMPLES);
  char oledStatus[20];
  snprintf(oledStatus, sizeof(oledStatus), "%u pairs, %u samp", pairs, SURVEY_SAMPLES);
  oled.showSplash("SURVEY", "Running...", oledStatus);

  uint8_t pairIdx = 0;
  for (uint8_t i = 0; i < N_ANCHORS; i++) {
    for (uint8_t j = i + 1; j < N_ANCHORS; j++) {
      uint8_t a = ANCHORS[i], b = ANCHORS[j];
      float   sum = 0.0f;
      uint16_t ok = 0;
      pairIdx++;

      char oledL1[20], oledL2[20];
      snprintf(oledL1, sizeof(oledL1), "%u/%u  %02X->%02X", pairIdx, pairs, a, b);
      Serial.printf("  Pair %u/%u  0x%02X -> 0x%02X:\n", pairIdx, pairs, a, b);

      for (uint16_t s = 0; s < SURVEY_SAMPLES; s++) {
        float dist, rxp;
        if (engine.surveyRequest(a, b, dist, rxp)) { sum += dist; ok++; }
        if ((s & 0xF) == 0xF) {
          snprintf(oledL2, sizeof(oledL2), "%u/%u ok=%u", s+1, SURVEY_SAMPLES, ok);
          oled.showSplash("SURVEY", oledL1, oledL2);
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
  oled.showSplash("SURVEY", "DONE", "See host PC");
}

static char g_bootMsg[40];   // reset forensics line, sent once the host link is up

// ── setup ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(300);
  ResetDiag::report(g_bootMsg, sizeof(g_bootMsg));

  // ── OLED first — must precede WiFi (same as TagWrover) ────────────────────
  oled.begin();
  oled.showSplash("TAG", "Starting...", "WiFi...");

  ChargeMode::begin();
  ChargeMode::checkAndSleep(&oled);   // short GPIO13-14 -> charge mode (deep sleep)

  // Own antenna delay: NVS-calibrated value overrides the sketch constant
  // (SETMYDELAY / HWCALIB_SELF persist here — retuning needs no reflash).
  prefs.begin("uwb", false);
  g_antDelay = prefs.getUShort("tagDelay", ANTENNA_DELAY);
  Serial.printf("[CAL] Tag antenna delay: %u%s\n", g_antDelay,
                g_antDelay == ANTENNA_DELAY ? " (sketch default)" : " (from NVS)");

  // ── WiFi / transport init ────────────────────────────────────────────────
#if defined(UWB_HOSTLINK_UDP)
  host.begin(WIFI_SSID, WIFI_PASS, HOST_IP, HOST_PORT);
  // WiFi association can glitch the I2C bus; re-init afterwards.
  Wire.end();
  delay(50);
  oled.begin();
  // ── UWB radio LAST — after WiFi/I2C settled ────────────────────────────────
  // Explicit SPI.begin() because WiFi + Wire bus teardowns above can leave the
  // ESP32 SPI peripheral in an undefined state on WROVER modules.
  // "DECA 01302001" in Serial → CS=4 correct; all zeros → wrong CS pin.
  SPI.begin(UWB_PIN_SCK, UWB_PIN_MISO, UWB_PIN_MOSI);
  engine.begin(TWR_TAG, TAG_ID, g_antDelay);
  scheduler.begin(&engine, ANCHORS, N_ANCHORS);
  ring.begin(&engine, TAG_ID, TAG_RING, RING_SIZE);
  engine.printDeviceId();
#else
  host.begin(115200);
  SPI.begin(UWB_PIN_SCK, UWB_PIN_MISO, UWB_PIN_MOSI);
  engine.begin(TWR_TAG, TAG_ID, g_antDelay);
  scheduler.begin(&engine, ANCHORS, N_ANCHORS);
  ring.begin(&engine, TAG_ID, TAG_RING, RING_SIZE);
  engine.printDeviceId();
#endif
  host.sendRaw(g_bootMsg);   // why did we last reset? (PANIC/WDT/BROWNOUT/...)

  Serial.printf("Tag 0x%02X ready — CS=%d  %u anchors  IMU:NO\n",
                TAG_ID, UWB_PIN_SS, N_ANCHORS);
  char l2[20];
  snprintf(l2, 20, "%u anch  IMU:NO", N_ANCHORS);
  oled.showSplash("TAG", "Ready", l2, "Ranging...");
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
  oled.showSplash("CALIBRATE", "Push delay...", "");
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
  char oledL1[20], oledL2[20];
  snprintf(oledL1, sizeof(oledL1), "Anch 0x%02X", anchorAddr);
  oled.showSplash("HW CALIB", oledL1, "Searching...");

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
    if ((it & 6) == 6) {
      snprintf(oledL2, sizeof(oledL2), "it%u d=%u", it + 1, mid);
      oled.showSplash("HW CALIB", oledL1, oledL2);
    }
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
  snprintf(oledL2, sizeof(oledL2), "d=%u %s", finalDelay, pushOk ? "OK" : "FAIL");
  oled.showSplash("HW CALIB", oledL1, oledL2);
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
  char oledL1[20], oledL2[20];
  snprintf(oledL1, sizeof(oledL1), "Self vs %02X", anchorAddr);
  oled.showSplash("SELF CALIB", oledL1, "Searching...");

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
    if ((it & 6) == 6) {
      snprintf(oledL2, sizeof(oledL2), "it%u d=%u", it + 1, mid);
      oled.showSplash("SELF CALIB", oledL1, oledL2);
    }
    if (meanMm > (float)trueDistMm) low = mid; else high = mid;
  }
  uint16_t finalDelay = (low + high) / 2;
  applyMyDelay(finalDelay);                    // apply + persist to NVS
  char buf[48];
  snprintf(buf, sizeof(buf), "HWCALIB_SELF_DONE,%u,%u\n", TAG_ID, finalDelay);
  host.sendRaw(buf);
  Serial.printf("[HWCAL-SELF] DONE  own delay=%u (NVS saved)\n", finalDelay);
  snprintf(oledL2, sizeof(oledL2), "d=%u OK", finalDelay);
  oled.showSplash("SELF CALIB", oledL1, oledL2);
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
void loop() {
  ChargeMode::checkAndSleep(&oled);   // jumper 13-14 enters charge mode at any time

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

  // ── UWB ranging sweep — multi-tag token ring: only sweep on our turn ──────
  static uint8_t nGood = 0;
  const bool myTurn = ring.poll();
  if (myTurn) {
    nGood = scheduler.sweep();
    host.sendSweep(millis(), TAG_ID, scheduler, nullptr);  // no IMU on this board
    ring.handoff();
  }

  // ── OLED status ────────────────────────────────────────────────────────────
  static uint32_t oledLast = 0;
  if (millis() - oledLast > 500) {
    oledLast = millis();
    updateOled(nGood);
  }
}
