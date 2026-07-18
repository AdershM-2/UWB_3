/*
 * Tag.ino - UWB RTLS tag (initiator).
 *
 * The tag ranges to every anchor in ANCHORS[] each sweep and streams the raw
 * distances to the MATLAB host. It NEVER solves position itself, so adding
 * anchors only means editing ANCHORS[] here and the matching AnchorConfig in
 * MATLAB - no algorithm change, and no firmware change once anchors are added.
 *
 * Transport: pick ONE of the two #defines below.
 *   UWB_HOSTLINK_SERIAL : stream over USB serial (simplest; tethered)
 *   UWB_HOSTLINK_UDP    : stream over WiFi UDP to the MATLAB PC
 */

// ---- choose ONE transport (compile-time) ----
#define UWB_HOSTLINK_SERIAL
// #define UWB_HOSTLINK_UDP
// #define UWB_USE_OLED

#include <UwbRtls.h>
#include <Preferences.h>

// >>>>>>>>>>>>>>>>> PER-BOARD CONFIGURATION <<<<<<<<<<<<<<<<<<
// Each tag board must have a unique TAG_ID within 0xF0â€“0xFE.
//   Board 1 (first tag):  TAG_ID = UWB_ADDR_TAG_BASE       (0xF0)
//   Board 2 (second tag): TAG_ID = UWB_ADDR_TAG_BASE + 1   (0xF1)
// ANTENNA_DELAY must be re-calibrated per board after any radio change.
static const uint8_t  TAG_ID        = UWB_ADDR_TAG_BASE;  // change per board
// Factory default only — an NVS-stored value (from SETMYDELAY / HWCALIB_SELF)
// overrides this at boot, so delay retuning never needs a reflash.
static const uint16_t ANTENNA_DELAY = UWB_DEFAULT_ANTENNA_DELAY;  // replace with calibrated value

// Anchor short addresses. ADD ANCHORS HERE (mirror in MATLAB AnchorConfig).
static const uint8_t ANCHORS[]  = { 0x01, 0x02, 0x03, 0x04, 0x05 };
static const uint8_t N_ANCHORS  = sizeof(ANCHORS) / sizeof(ANCHORS[0]);

// Multi-tag token ring: the ordered list of ALL tag addresses sharing the
// channel. MUST be identical on every tag board (default from UwbConfig.h).
static const uint8_t TAG_RING[] = UWB_TAG_RING_INIT;
static const uint8_t RING_SIZE  = sizeof(TAG_RING) / sizeof(TAG_RING[0]);

#if defined(UWB_HOSTLINK_UDP)
static const char*   WIFI_SSID = "your-ssid";
static const char*   WIFI_PASS = "your-pass";
static IPAddress     HOST_IP(192, 168, 1, 100);   // the MATLAB PC's IP
static const uint16_t HOST_PORT = 5005;
#endif
// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// Number of ranging samples averaged per anchor pair during self-survey.
static const uint16_t SURVEY_SAMPLES = 100;

// Hardware antenna-delay calibration (GUI-triggered binary search per anchor).
static const uint8_t  CALIB_ITERS    = 14;
static const uint16_t CALIB_SAMPLES  = 50;
static const uint16_t CALIB_DELAY_LO = 15800;
static const uint16_t CALIB_DELAY_HI = 16900;

TwrEngine    engine;
UwbScheduler scheduler;
TagRing      ring;    // multi-tag coordination (one tag transmits at a time)
HostLink     host;
OledStatus   oled;
SensorImu    imu;     // stub today; populates the IMU,... packet tail later
Preferences  prefs;

static uint16_t g_antDelay = ANTENNA_DELAY;  // active own delay (NVS-backed)

static char g_bootMsg[40];   // reset forensics line, sent once the host link is up

void setup() {
  Serial.begin(115200);
  delay(200);
  ResetDiag::report(g_bootMsg, sizeof(g_bootMsg));
  oled.begin();
  ChargeMode::begin();
  ChargeMode::checkAndSleep(&oled);   // short GPIO13-14 -> charge mode (deep sleep)

  // Own antenna delay: NVS-calibrated value overrides the sketch constant.
  prefs.begin("uwb", false);
  g_antDelay = prefs.getUShort("tagDelay", ANTENNA_DELAY);
  Serial.printf("[CAL] Tag antenna delay: %u%s\n", g_antDelay,
                g_antDelay == ANTENNA_DELAY ? " (sketch default)" : " (from NVS)");

  engine.begin(TWR_TAG, TAG_ID, g_antDelay);
  scheduler.begin(&engine, ANCHORS, N_ANCHORS);
  ring.begin(&engine, TAG_ID, TAG_RING, RING_SIZE);

#if defined(UWB_HOSTLINK_UDP)
  host.begin(WIFI_SSID, WIFI_PASS, HOST_IP, HOST_PORT);
#else
  host.begin(115200);
#endif
  host.sendRaw(g_bootMsg);   // why did we last reset? (PANIC/WDT/BROWNOUT/...)

  imu.begin();    // false until the BNO085 driver is implemented
  oled.begin();
  engine.printDeviceId();
  Serial.printf("Tag 0x%02X ready, %u anchors, ring of %u\n",
                TAG_ID, N_ANCHORS, RING_SIZE);
}

// Anchor self-survey â€” send "SURVEY\n" over serial to trigger.
// Wire format out: SURVEY_BEGIN,v1,<pairs> / SURVEY,v1,<src>,<dst>,<mm>,<ok> / SURVEY_DONE,v1
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
        if ((s & 0xF) == 0xF)
          Serial.printf("    [%u/%u ok=%u]\n", s + 1, SURVEY_SAMPLES, ok);
        delay(5);
      }

      if (ok >= SURVEY_SAMPLES / 4) {
        uint32_t mm = (uint32_t)lroundf(sum / ok * 1000.0f);
        snprintf(buf, sizeof(buf), "SURVEY,v1,%u,%u,%lu,%u\n", a, b, mm, ok);
        host.sendRaw(buf);
        Serial.printf("  => %.3f m  (%u/%u ok)\n", sum / ok, ok, SURVEY_SAMPLES);
      } else {
        Serial.printf("  FAILED (ok=%u/%u)\n", ok, SURVEY_SAMPLES);
      }
    }
  }
  host.sendRaw("SURVEY_DONE,v1\n");
  Serial.println("Survey complete.");
}

static void handleHwCalib(const char* cmd) {
  unsigned int aid, trueDistMm;
  if (sscanf(cmd, "HWCALIB,%u,%u", &aid, &trueDistMm) != 2) {
    host.sendRaw("HWCALIB_FAIL,0,parse_error\n");
    return;
  }
  uint8_t anchorAddr = (uint8_t)aid;
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
    if (meanMm > (float)trueDistMm) low = mid; else high = mid;
  }
  uint16_t finalDelay = (low + high) / 2;
  bool pushOk = engine.pushAntDelay(anchorAddr, finalDelay, 3);
  char buf[48];
  snprintf(buf, sizeof(buf), pushOk ? "HWCALIB_DONE,%u,%u\n"
                                    : "HWCALIB_FAIL,%u,final_push\n",
           aid, finalDelay);
  host.sendRaw(buf);
}

// Own antenna-delay control (tag-side twin of the anchor NVS flow).
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
// of pushing to the anchor — per-tag bias must not be absorbed into anchor NVS.
static void handleHwCalibSelf(const char* cmd) {
  unsigned int tid, aid, trueDistMm;
  if (sscanf(cmd, "HWCALIB_SELF,%u,%u,%u", &tid, &aid, &trueDistMm) != 3) {
    host.sendRaw("HWCALIB_SELF_FAIL,0,parse_error\n");
    return;
  }
  if ((uint8_t)tid != TAG_ID) return;   // addressed to another tag
  uint8_t anchorAddr = (uint8_t)aid;
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
    if (meanMm > (float)trueDistMm) low = mid; else high = mid;
  }
  uint16_t finalDelay = (low + high) / 2;
  applyMyDelay(finalDelay);                    // apply + persist to NVS
  char buf[48];
  snprintf(buf, sizeof(buf), "HWCALIB_SELF_DONE,%u,%u\n", TAG_ID, finalDelay);
  host.sendRaw(buf);
}

static void dispatchCmd(const char* cmd) {
  if (strcmp(cmd, "SURVEY") == 0)
    runSurvey();
  else if (strncmp(cmd, "HWCALIB_SELF,", 13) == 0)
    handleHwCalibSelf(cmd);
  else if (strncmp(cmd, "HWCALIB,", 8) == 0)
    handleHwCalib(cmd);
  else if (strncmp(cmd, "SETMYDELAY,", 11) == 0)
    handleSetMyDelay(cmd);
  else if (strncmp(cmd, "GETMYDELAY,", 11) == 0)
    handleGetMyDelay(cmd);
}

void loop() {
  ChargeMode::checkAndSleep(&oled);   // jumper 13-14 enters charge mode at any time

  // Non-blocking serial command check.
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

  // Multi-tag token ring: sweep + stream ONLY on our turn, then pass the token.
  static uint8_t good = 0;
  if (ring.poll()) {
    good = scheduler.sweep();
    ImuSample s;
    bool hasImu = imu.read(s);     // false in the stub
    host.sendSweep(millis(), TAG_ID, scheduler, hasImu ? &s : nullptr);
    ring.handoff();
  }

  static uint32_t oledLast = 0;
  if (millis() - oledLast > 500) {
    oledLast = millis();
    char l0[24], l1[24];
    snprintf(l0, sizeof(l0), "TAG sweep %lu", (unsigned long)scheduler.sweepSeq());
    snprintf(l1, sizeof(l1), "%u/%u anchors", good, N_ANCHORS);
    oled.show(l0, l1);
  }
}
