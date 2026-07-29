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

// ---------------------------------------------------------------------------
// Wired master/slave TDMA (TagLink) — the alternative to the over-air token
// ring above. Selected per sketch:
//
//     #define UWB_COORD_WIRE          // instead of the default UWB_COORD_RING
//     #define UWB_WIRE_ROLE_MASTER    // on the IMU board only
//
// The two tag boards are cross-wired on UART2 (A.TX -> B.RX, B.TX -> A.RX, plus
// a SHARED GROUND — the link does not work without it). The master grants the
// slave an air slot over the wire, so exactly one DW1000 transmits at any
// instant and the air is never contended.
//
// Wiring (both boards): TAGLINK_PIN_TX -> peer's TAGLINK_PIN_RX, plus common
// ground; keep the leads short and twisted. GPIO 22 (output-capable) and GPIO
// 35 (input-only — fine for RX) are free on the TagWrover map (SS=4, OLED
// 32/33, BNO 25/26) and on the display-board map (SS=21, OLED 4/5). Never use
// 16/17 (WROVER PSRAM), 13/14 (charge jumper), or 12 (strapping — MTDI must be
// LOW at boot on 3.3 V flash parts, so a UART there stops the board booting).
// ---------------------------------------------------------------------------
// "Pick exactly ONE" is enforced, not merely requested. Only UWB_COORD_WIRE is
// ever tested in code (UWB_COORD_RING is the documented name for the default),
// so defining both would silently give you WIRE while the sketch reads as RING.
#if defined(UWB_COORD_WIRE) && defined(UWB_COORD_RING)
#error "Define UWB_COORD_WIRE or UWB_COORD_RING, not both - comment one out"
#endif

#ifndef TAGLINK_PIN_TX
#define TAGLINK_PIN_TX  22
#endif
#ifndef TAGLINK_PIN_RX
#define TAGLINK_PIN_RX  35
#endif
#ifndef TAGLINK_BAUD
#define TAGLINK_BAUD    921600   // 50 cm wire: comfortable; ~11 µs per byte
#endif

// Adaptive slot budget: 1.5 x EMA of the slave's realised sweep, clamped.
// SIZING: a failing rangeTo() costs waitSent(20) + waitReceived(45) +
// waitSent(30) + waitReceived(30) = 125 ms, so an all-fail sweep of N anchors
// takes 125*N ms — 625 ms at N=5. The CEIL must exceed that or the budget can
// never grow enough to cover a bad sweep and the master declares a perfectly
// healthy slave absent. The FLOOR stops a run of lucky fast sweeps from
// shrinking the budget below a legitimate worst case.
#ifndef TAGLINK_BUDGET_INIT_MS
#define TAGLINK_BUDGET_INIT_MS   700
#endif
#ifndef TAGLINK_BUDGET_FLOOR_MS
#define TAGLINK_BUDGET_FLOOR_MS  150
#endif
#ifndef TAGLINK_BUDGET_CEIL_MS
#define TAGLINK_BUDGET_CEIL_MS   700
#endif

// Slave HELLO cadence: fast until the master ACKs, then a slow heartbeat. This
// is what lets the slave hot-join or rejoin at any time without either board
// rebooting.
#ifndef TAGLINK_HELLO_FAST_MS
#define TAGLINK_HELLO_FAST_MS    100
#endif
#ifndef TAGLINK_HELLO_BEAT_MS
#define TAGLINK_HELLO_BEAT_MS    500
#endif

// Consecutive slot timeouts before the master declares the slave absent and
// runs solo. It keeps listening for HELLO throughout.
#ifndef TAGLINK_ABSENT_AFTER
#define TAGLINK_ABSENT_AFTER     3
#endif

// Catch a pin clash at COMPILE time rather than as a mystery at the bench. This
// matters most for UWB_PIN_SS, which defaults to 21 on the Makerfabs "Pro with
// Display" variant — boards that keep that default must override TAGLINK_PIN_RX
// (and rewire) before enabling WIRE mode.
#if defined(UWB_COORD_WIRE)
  #if (TAGLINK_PIN_RX == UWB_PIN_SS)   || (TAGLINK_PIN_TX == UWB_PIN_SS)   || \
      (TAGLINK_PIN_RX == UWB_PIN_SCK)  || (TAGLINK_PIN_TX == UWB_PIN_SCK)  || \
      (TAGLINK_PIN_RX == UWB_PIN_MISO) || (TAGLINK_PIN_TX == UWB_PIN_MISO) || \
      (TAGLINK_PIN_RX == UWB_PIN_MOSI) || (TAGLINK_PIN_TX == UWB_PIN_MOSI) || \
      (TAGLINK_PIN_RX == UWB_PIN_RST)  || (TAGLINK_PIN_TX == UWB_PIN_RST)  || \
      (TAGLINK_PIN_RX == UWB_PIN_IRQ)  || (TAGLINK_PIN_TX == UWB_PIN_IRQ)  || \
      (TAGLINK_PIN_RX == OLED_PIN_SDA) || (TAGLINK_PIN_TX == OLED_PIN_SDA) || \
      (TAGLINK_PIN_RX == OLED_PIN_SCL) || (TAGLINK_PIN_TX == OLED_PIN_SCL)
    #error "TAGLINK_PIN_RX/TX collides with an SPI, DW1000 or OLED pin - see the pin map above"
  #endif
  #if (TAGLINK_PIN_RX == TAGLINK_PIN_TX)
    #error "TAGLINK_PIN_RX and TAGLINK_PIN_TX must differ"
  #endif
  #if (TAGLINK_PIN_TX >= 34)
    #error "TAGLINK_PIN_TX is an input-only pin (34-39) - it cannot transmit"
  #endif
#endif


#endif // UWBRTLS_UWBCONFIG_H
