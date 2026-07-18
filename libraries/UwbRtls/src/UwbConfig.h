/*
 * UwbConfig.h - Shared radio / board configuration for the UwbRtls library.
 *
 * These settings MUST be identical on every board in the network (tag + all
 * anchors), with the sole exception of per-device values that each sketch sets
 * itself: the device short address and the calibrated antenna delay.
 *
 * Board: Makerfabs "ESP32 UWB Pro with Display" (Decawave DW1000 / BU01).
 */
#ifndef UWBRTLS_UWBCONFIG_H
#define UWBRTLS_UWBCONFIG_H

#include <Arduino.h>
#include "dw1000/DW1000.h"

// ---------------------------------------------------------------------------
// Pin map defaults — Makerfabs ESP32 UWB Pro with Display.
//
// Board variants use different CS (SS) pins:
//   ESP32 UWB Pro WITH display  : UWB_PIN_SS = 21  (default below)
//   ESP32 UWB  (no Pro/display) : UWB_PIN_SS = 4
//
// A sketch can override any pin BEFORE including UwbRtls.h:
//   #define UWB_PIN_SS 4
//   #include <UwbRtls.h>
// ---------------------------------------------------------------------------
#ifndef UWB_PIN_SCK
#define UWB_PIN_SCK   18
#endif
#ifndef UWB_PIN_MISO
#define UWB_PIN_MISO  19
#endif
#ifndef UWB_PIN_MOSI
#define UWB_PIN_MOSI  23
#endif
#ifndef UWB_PIN_SS
#define UWB_PIN_SS    21   // SPI chip select — override to 4 for non-Pro boards
#endif
#ifndef UWB_PIN_RST
#define UWB_PIN_RST   27   // DW1000 reset
#endif
#ifndef UWB_PIN_IRQ
#define UWB_PIN_IRQ   34   // DW1000 interrupt (input-only pin, fine for IRQ)
#endif

#ifndef OLED_PIN_SDA
#define OLED_PIN_SDA   4
#endif
#ifndef OLED_PIN_SCL
#define OLED_PIN_SCL   5
#endif

// ---------------------------------------------------------------------------
// Radio profile - range accuracy mode. Same on every board. (Phase 1.0)
//   MODE_LONGDATA_RANGE_ACCURACY = 110 kbps, 64 MHz PRF, 2048-symbol preamble.
//   64 MHz PRF sharpens the CIR autocorrelation peak and rejects multipath
//   better than the 16 MHz LOWPOWER variants. Long preamble maximises
//   sensitivity at the cost of ~10 Hz sweep rate.
//   Phase 1.3B (MODE_SHORTDATA_FAST_ACCURACY, 6.8 Mb/s) was attempted but
//   reverted due to hardware incompatibility — revisit after diagnosis.
// ---------------------------------------------------------------------------
#define UWB_RADIO_MODE   DW1000.MODE_LONGDATA_RANGE_ACCURACY
#define UWB_CHANNEL      DW1000.CHANNEL_5

// Reply delay used for the delayed transmits in two-way ranging (microseconds).
// History: 7000 us (16 MHz LOWPOWER) → 6000 us (Phase 1.0, 64 MHz ACCURACY) →
//          5000 us (Phase 1.3A, tighter margin, well-validated).
// At 110 kbps the frame takes ~2 ms on-air; ESP32 SPI + DW1000 processing
// needs ~1000-1200 us; 5000 us leaves a comfortable ~1800 us margin.
#define UWB_REPLY_DELAY_US   5000

// Default antenna delay (DW1000 ticks). Each board overrides this with its own
// CALIBRATED value (see examples/AntennaCalibration). 16384 is the chip reset
// default; tuned values typically land in 16450..16650.
#define UWB_DEFAULT_ANTENNA_DELAY  16384

// ---------------------------------------------------------------------------
// Addressing (1-byte short addresses, our own scheme - not 802.15.4).
//   0x00          : reserved / invalid
//   0x01 .. 0xEF  : anchors
//   0xF0 .. 0xFE  : tags
//   0xFF          : broadcast (reserved for future multi-tag announce/slotting)
// ---------------------------------------------------------------------------
#define UWB_ADDR_INVALID    0x00
#define UWB_ADDR_BROADCAST  0xFF
#define UWB_ADDR_TAG_BASE   0xF0   // first tag = 0xF0

// Network id (shared). Frame filtering is done in software, so this is mostly
// cosmetic, but we keep it consistent across the fleet.
#define UWB_NETWORK_ID  0xDECA

// ---------------------------------------------------------------------------
// Multi-tag token ring (TagRing). Only one tag transmits at a time: each tag
// sweeps all anchors, then broadcasts a SLOT_GRANT token handing the turn to
// the next tag in the ring. A grantee confirms with an ANNOUNCE (claim) so the
// grantor knows fast whether the slot is occupied. Timeouts below only bound
// RECOVERY latency (a tag joining/leaving/dying); steady-state cadence is
// driven by the tokens themselves, not by these timers.
//
//   CLAIM   : a freshly-granted tag must ANNOUNCE within this, else it is
//             presumed absent and the grant is passed to the next position.
//             Must exceed token TX + the grantee's loop latency (~few ms).
//   COMPLETE: an ANNOUNCEd holder must finish its sweep + hand off within this.
//             Must exceed the worst-case full sweep time.
//   RECLAIM : total-ring-silence backstop. If a waiting tag hears NOTHING for
//             RECLAIM_BASE + myIndex*RECLAIM_STAGGER it claims the turn. The
//             per-index stagger breaks symmetry on simultaneous boot / dead ring
//             (lowest live index reclaims first).
// ---------------------------------------------------------------------------
#ifndef UWB_RING_MAX
#define UWB_RING_MAX  8     // max tag positions in the ring (RAM-bounded)
#endif
// Default ring membership — the ordered list of tag short addresses that share
// the channel. MUST be identical on every tag board (like UWB_NETWORK_ID).
// List ONLY the tags you actually deploy: an absent ring member costs one
// CLAIM_TIMEOUT per cycle while the ring probes it. Add 0xF2/0xF3 here (on every
// tag) when you add those boards. A sketch may override before #include <UwbRtls.h>.
#ifndef UWB_TAG_RING_INIT
#define UWB_TAG_RING_INIT  { 0xF0, 0xF1 }
#endif
#ifndef UWB_RING_CLAIM_TIMEOUT_MS
#define UWB_RING_CLAIM_TIMEOUT_MS     30
#endif
#ifndef UWB_RING_COMPLETE_TIMEOUT_MS
#define UWB_RING_COMPLETE_TIMEOUT_MS  400
#endif
#ifndef UWB_RING_RECLAIM_BASE_MS
#define UWB_RING_RECLAIM_BASE_MS      500
#endif
#ifndef UWB_RING_RECLAIM_STAGGER_MS
#define UWB_RING_RECLAIM_STAGGER_MS   60
#endif

#endif // UWBRTLS_UWBCONFIG_H
