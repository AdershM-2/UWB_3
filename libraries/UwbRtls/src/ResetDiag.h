#pragma once
// ---------------------------------------------------------------------------
// ResetDiag — boot forensics for the repeated-reset investigation.
//
// Field symptom (2026-07-01 notes): both tags resetting again and again with
// the dual-tag ring active. This header distinguishes the three suspect
// classes at zero cost:
//   PANIC     -> firmware crash (exception/assert) — get the backtrace over USB
//   *_WDT     -> a loop stopped yielding (watchdog starvation)
//   BROWNOUT  -> supply dipped (battery/USB can't carry WiFi TX + DW1000 + OLED)
//
// Usage: call ResetDiag::report(buf, len) FIRST thing after Serial.begin();
// it prints "[BOOT] reset_reason=... boot#N" and fills buf with a
// "BOOT,<reason>,<n>" line to forward over HostLink once the link is up, so
// field logs on the PC capture every reboot without a USB cable attached.
//
// The boot counter lives in RTC_NOINIT memory: it survives every reset type
// except full power-off, so a crash-loop shows as boot#2,3,4,... while
// power-cycles restart at 1.
// ---------------------------------------------------------------------------
#include <Arduino.h>
#include <esp_system.h>

namespace ResetDiag {

RTC_NOINIT_ATTR static uint32_t bootCount;  // per-TU is fine: only the sketch calls report()

inline const char* reasonStr(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON:   return "POWERON";
    case ESP_RST_EXT:       return "EXT_PIN";
    case ESP_RST_SW:        return "SW_RESTART";
    case ESP_RST_PANIC:     return "PANIC";
    case ESP_RST_INT_WDT:   return "INT_WDT";
    case ESP_RST_TASK_WDT:  return "TASK_WDT";
    case ESP_RST_WDT:       return "OTHER_WDT";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP_WAKE";
    case ESP_RST_BROWNOUT:  return "BROWNOUT";
    case ESP_RST_SDIO:      return "SDIO";
    default:                return "UNKNOWN";
  }
}

inline esp_reset_reason_t report(char* out = nullptr, size_t outLen = 0) {
  esp_reset_reason_t r = esp_reset_reason();
  if (r == ESP_RST_POWERON) bootCount = 0;   // RTC RAM is garbage after power loss
  bootCount++;
  Serial.printf("[BOOT] reset_reason=%s boot#%lu\n",
                reasonStr(r), (unsigned long)bootCount);
  if (out && outLen)   // trailing \n: HostLink::sendRaw sends lines as-is
    snprintf(out, outLen, "BOOT,%s,%lu\n", reasonStr(r), (unsigned long)bootCount);
  return r;
}

}  // namespace ResetDiag
