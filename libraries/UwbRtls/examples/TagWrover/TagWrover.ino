/*
 * TagWrover.ino  â€“  UWB tag for a generic ESP32-WROVER + DW1000 module
 *                   with external SSD1306 OLED and BNO085 IMU.
 *
 * Reference copy (no OLED define, serial transport, generic credentials).
 * The active flash target with WiFi credentials lives in sketches/TagWrover/.
 *
 * Peripheral map
 * â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
 *  DW1000 (SPI) : SCK=18  MISO=19  MOSI=23  CS=4   RST=27  IRQ=34
 *  SSD1306 OLED : SDA=4   SCL=5    (Wire,  I2C address 0x3C or 0x3D)
 *  BNO085 IMU   : SDA=16  SCL=17   (Wire1, I2C address 0x4A)
 *
 * âš   GPIO 16/17 are used for PSRAM on most ESP32-WROVER modules.
 *    If PSRAM is enabled in the Arduino IDE, change BNO_SDA/BNO_SCL to 25/26
 *    or 32/33 and rewire accordingly.
 *
 * Required libraries:
 *   Adafruit BNO08x, Adafruit SSD1306, Adafruit GFX Library
 */

// â”€â”€ Transport â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
#define UWB_HOSTLINK_SERIAL
// #define UWB_HOSTLINK_UDP
// #define UWB_USE_OLED   // uncomment if OLED is connected

// â”€â”€ Board pin override â€” ESP32 UWB (no display / non-Pro) uses CS=4 not 21 â”€â”€
#define UWB_PIN_SS 4

#include <UwbRtls.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <Preferences.h>

// â”€â”€ Configuration â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
static const uint8_t  TAG_ID        = UWB_ADDR_TAG_BASE;
// Factory default only — an NVS-stored value (from SETMYDELAY / HWCALIB_SELF)
// overrides this at boot, so delay retuning never needs a reflash.
static const uint16_t ANTENNA_DELAY = UWB_DEFAULT_ANTENNA_DELAY;

static const uint8_t ANCHORS[]  = { 0x01, 0x02, 0x03, 0x04, 0x05 };
static const uint8_t N_ANCHORS  = sizeof(ANCHORS) / sizeof(ANCHORS[0]);

// Multi-tag token ring: ordered list of ALL tag addresses sharing the channel.
// MUST be identical on every tag board (default from UwbConfig.h).
static const uint8_t TAG_RING[] = UWB_TAG_RING_INIT;
static const uint8_t RING_SIZE  = sizeof(TAG_RING) / sizeof(TAG_RING[0]);

#if defined(UWB_HOSTLINK_UDP)
static const char*    WIFI_SSID = "your-ssid";
static const char*    WIFI_PASS = "";
static IPAddress      HOST_IP(192, 168, 1, 100);
static const uint16_t HOST_PORT  = 4100;
#endif

#define BNO_SDA   16
#define BNO_SCL   17
#define BNO_ADDR  0x4A

static const uint16_t SURVEY_SAMPLES = 100;

// Hardware antenna-delay calibration (GUI-triggered binary search per anchor).
static const uint8_t  CALIB_ITERS    = 14;
static const uint16_t CALIB_SAMPLES  = 50;
static const uint16_t CALIB_DELAY_LO = 15800;
static const uint16_t CALIB_DELAY_HI = 16900;

// â”€â”€ Objects â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
TwrEngine    engine;
UwbScheduler scheduler;
TagRing      ring;    // multi-tag coordination (one tag transmits at a time)
HostLink     host;
OledStatus   oled;
Preferences  prefs;

static uint16_t g_antDelay = ANTENNA_DELAY;  // active own delay (NVS-backed)

Adafruit_BNO08x bno085(-1);
static bool     imuPresent = false;

static struct {
  float   qw=1, qx=0, qy=0, qz=0;
  float   ax=0, ay=0, az=0;
  float   gx=0, gy=0, gz=0;
  float   roll=0, pitch=0, yaw=0;
  uint8_t status = 0;
  bool    valid  = false;
} imuData;

// â”€â”€ IMU helpers â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€

static void quatToRPY(float qw, float qx, float qy, float qz,
                      float& roll, float& pitch, float& yaw) {
  float sinr = 2.0f*(qw*qx + qy*qz);
  float cosr = 1.0f - 2.0f*(qx*qx + qy*qy);
  roll = atan2f(sinr, cosr) * 180.0f / (float)PI;

  float sinp = 2.0f*(qw*qy - qz*qx);
  pitch = (fabsf(sinp) >= 1.0f) ? copysignf(90.0f, sinp)
                                 : asinf(sinp) * 180.0f / (float)PI;

  float siny = 2.0f*(qw*qz + qx*qy);
  float cosy = 1.0f - 2.0f*(qy*qy + qz*qz);
  yaw = atan2f(siny, cosy) * 180.0f / (float)PI;
}

static void setImuReports() {
  bno085.enableReport(SH2_ROTATION_VECTOR,      10000);
  bno085.enableReport(SH2_LINEAR_ACCELERATION,  10000);
  bno085.enableReport(SH2_GYROSCOPE_CALIBRATED, 10000);
}

static void pollImu() {
  if (!imuPresent) return;
  if (bno085.wasReset()) {
    Serial.println("[IMU] reset â€” re-enabling reports");
    setImuReports();
  }
  sh2_SensorValue_t val;
  while (bno085.getSensorEvent(&val)) {
    switch (val.sensorId) {
      case SH2_ROTATION_VECTOR:
        imuData.qw     = val.un.rotationVector.real;
        imuData.qx     = val.un.rotationVector.i;
        imuData.qy     = val.un.rotationVector.j;
        imuData.qz     = val.un.rotationVector.k;
        imuData.status = val.status;
        quatToRPY(imuData.qw, imuData.qx, imuData.qy, imuData.qz,
                  imuData.roll, imuData.pitch, imuData.yaw);
        imuData.valid  = true;
        break;
      case SH2_LINEAR_ACCELERATION:
        imuData.ax = val.un.linearAcceleration.x;
        imuData.ay = val.un.linearAcceleration.y;
        imuData.az = val.un.linearAcceleration.z;
        break;
      case SH2_GYROSCOPE_CALIBRATED:
        imuData.gx = val.un.gyroscope.x;
        imuData.gy = val.un.gyroscope.y;
        imuData.gz = val.un.gyroscope.z;
        break;
    }
  }
}

static void printImuSerial() {
  static uint32_t lastMs = 0;
  if (millis() - lastMs < 200) return;
  lastMs = millis();
  if (!imuPresent) { Serial.println("[IMU] not present"); return; }
  if (!imuData.valid) { Serial.println("[IMU] waiting..."); return; }
  Serial.printf("[IMU] R:%+7.2f  P:%+7.2f  Y:%+7.2f deg    "
                "Ax:%+7.3f  Ay:%+7.3f  Az:%+7.3f m/s2    "
                "Gx:%+6.3f  Gy:%+6.3f  Gz:%+6.3f rad/s  conf=%u\n",
                imuData.roll, imuData.pitch, imuData.yaw,
                imuData.ax,   imuData.ay,   imuData.az,
                imuData.gx,   imuData.gy,   imuData.gz,
                (unsigned)imuData.status);
}

static void updateOled(uint8_t nGood) {
  char l0[22], l1[22], l2[22], l3[22];
  snprintf(l0, 22, "TAG:%02X  %u/%u OK", TAG_ID, nGood, N_ANCHORS);
  if (!imuPresent) {
    snprintf(l1, 22, "IMU not found");
    l2[0] = l3[0] = '\0';
  } else if (!imuData.valid) {
    snprintf(l1, 22, "IMU init..."); l2[0] = l3[0] = '\0';
  } else {
    snprintf(l1, 22, "R:%+6.1f P:%+6.1f",   imuData.roll,  imuData.pitch);
    snprintf(l2, 22, "Y:%+6.1f deg",          imuData.yaw);
    snprintf(l3, 22, "a:%+5.2f%+5.2f%+5.2f", imuData.ax, imuData.ay, imuData.az);
  }
  oled.show(l0, l1, l2, l3);
}

// â”€â”€ Self-survey â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
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
      float sum = 0.0f; uint16_t ok = 0;
      pairIdx++;
      Serial.printf("  Pair %u/%u  0x%02X -> 0x%02X:\n", pairIdx, pairs, a, b);
      for (uint16_t s = 0; s < SURVEY_SAMPLES; s++) {
        float dist, rxp;
        if (engine.surveyRequest(a, b, dist, rxp)) { sum += dist; ok++; }
        if ((s & 0xF) == 0xF)
          Serial.printf("    [%u/%u ok=%u]\n", s+1, SURVEY_SAMPLES, ok);
        delay(5);
      }
      if (ok >= SURVEY_SAMPLES / 4) {
        uint32_t mm = (uint32_t)lroundf(sum / ok * 1000.0f);
        snprintf(buf, sizeof(buf), "SURVEY,v1,%u,%u,%lu,%u\n", a, b, mm, ok);
        host.sendRaw(buf);
        Serial.printf("  => %.3f m  (%u/%u ok)\n", sum/ok, ok, SURVEY_SAMPLES);
      } else {
        Serial.printf("  FAILED (ok=%u/%u)\n", ok, SURVEY_SAMPLES);
      }
    }
  }
  host.sendRaw("SURVEY_DONE,v1\n");
  Serial.println("Survey complete.");
}

static char g_bootMsg[40];   // reset forensics line, sent once the host link is up

// â”€â”€ setup â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
void setup() {
  // OLED first â€” must precede WiFi (same as Tag.ino).
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

  Wire1.begin(BNO_SDA, BNO_SCL);
  if (bno085.begin_I2C(BNO_ADDR, &Wire1)) {
    imuPresent = true;
    setImuReports();
  }

#if defined(UWB_HOSTLINK_UDP)
  host.begin(WIFI_SSID, WIFI_PASS, HOST_IP, HOST_PORT);
  Wire.end(); delay(50); oled.begin();
  if (imuPresent) { Wire1.end(); delay(20); Wire1.begin(BNO_SDA, BNO_SCL); setImuReports(); }
  // DW1000 LAST â€” after WiFi/I2C settled. Explicit SPI.begin() re-arms the bus
  // after WiFi + Wire teardowns, which can leave it undefined on WROVER modules.
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

  Serial.printf("Tag 0x%02X ready â€” CS=%d  %u anchors  IMU:%s\n",
                TAG_ID, UWB_PIN_SS, N_ANCHORS, imuPresent ? "YES" : "NO");
}

// â”€â”€ Antenna delay push relay â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
static void handleSetAntDelay(const char* cmd) {
  unsigned int aid = 0, ticks = 0;
  if (sscanf(cmd, "SETANTDELAY,%u,%u", &aid, &ticks) != 2) {
    Serial.println("[CAL] SETANTDELAY parse error â€” expected SETANTDELAY,<id>,<ticks>");
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

// â”€â”€ loop â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
void loop() {
  ChargeMode::checkAndSleep(&oled);   // jumper 13-14 enters charge mode at any time

  static char udpCmd[32];
  if (host.receiveCmd(udpCmd, sizeof(udpCmd)) > 0)
    dispatchCmd(udpCmd);

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

  pollImu();   // always drain the BNO085 FIFO, even while waiting our turn

  // Multi-tag token ring: sweep + stream ONLY on our turn, then hand off.
  static uint8_t nGood = 0;
  if (ring.poll()) {
    nGood = scheduler.sweep();
    pollImu();   // capture data accumulated during the ranging wait

    ImuSample imuSamp;
    if (imuPresent && imuData.valid) {
      imuSamp.valid  = true;
      imuSamp.status = imuData.status;
      imuSamp.qw = imuData.qw;  imuSamp.qx = imuData.qx;
      imuSamp.qy = imuData.qy;  imuSamp.qz = imuData.qz;
      imuSamp.ax = imuData.ax;  imuSamp.ay = imuData.ay;  imuSamp.az = imuData.az;
      imuSamp.gx = imuData.gx;  imuSamp.gy = imuData.gy;  imuSamp.gz = imuData.gz;
    }

    host.sendSweep(millis(), TAG_ID, scheduler,
                   (imuPresent && imuData.valid) ? &imuSamp : nullptr);
    ring.handoff();
  }

  printImuSerial();
  static uint32_t oledLast = 0;
  if (millis() - oledLast > 500) {
    oledLast = millis();
    updateOled(nGood);
  }
}
