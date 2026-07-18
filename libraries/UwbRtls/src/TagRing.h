/*
 * TagRing.h - Multi-tag token-passing ring for the UWB RTLS tag role.
 *
 * Problem it solves: with the single-tag firmware, two tags free-run their
 * anchor sweeps simultaneously. Their frames collide, the anchor (which tracks a
 * single _peer) can follow only one exchange, and the scheduler's dead-anchor
 * backoff amplifies the imbalance until one tag captures every anchor and the
 * other goes silent ("the anchors drop the old tag and join the new one").
 *
 * Mechanism: only ONE tag transmits at a time. On its turn a tag sweeps all
 * anchors (UwbScheduler) and streams the result, then broadcasts a SLOT_GRANT
 * token handing the next turn to the next tag in a fixed ring. The grantee
 * confirms with an ANNOUNCE (claim) so the grantor learns within a few ms whether
 * the slot is occupied; an unclaimed grant is passed on, skipping an absent tag.
 * A total-silence backstop (staggered by ring index, so the lowest live index
 * wins) reclaims the turn on cold start or if the active holder dies.
 *
 * No anchor change is needed: anchors stay pure responders and exactly one tag
 * ever polls them at a time, so the single-_peer limit never triggers. The token
 * and announce frames are broadcast; anchors receive and harmlessly ignore them
 * (unknown-type fall-through in TwrEngine::serviceResponder).
 *
 * The ring list MUST be identical on every tag board (see UWB_TAG_RING_INIT).
 * Each board still needs its own unique TAG_ID (== its short address).
 *
 * Usage (tag loop):
 *     if (ring.poll()) {                 // true only when it's our turn
 *       scheduler.sweep();               // existing ranging, unchanged
 *       host.sendSweep(...);             // existing streaming, unchanged
 *       ring.handoff();                  // pass the token to the next tag
 *     }
 */
#ifndef UWBRTLS_TAGRING_H
#define UWBRTLS_TAGRING_H

#include <Arduino.h>
#include "UwbConfig.h"
#include "UwbFrame.h"
#include "TwrEngine.h"

class TagRing {
public:
  // engine:   a TwrEngine already begun in TWR_TAG role.
  // myAddr:   this tag's short address (== TAG_ID); MUST appear in ring[].
  // ring:     ordered list of all tag short addresses that share the channel.
  // ringSize: number of entries in ring[] (clamped to UWB_RING_MAX).
  void begin(TwrEngine* engine, uint8_t myAddr, const uint8_t* ring, uint8_t ringSize);

  // Call every loop() iteration while NOT sweeping. Non-blocking. Returns true
  // exactly when it becomes THIS tag's turn; the caller must then sweep + stream
  // and call handoff().
  bool poll();

  // Call once after the sweep+stream that followed poll()==true. Broadcasts the
  // token to the next ring position and returns to the waiting state.
  void handoff();

  // Diagnostics.
  uint8_t holder()   const { return _holder; }       // believed turn-holder
  bool    isMyTurn() const { return _state == MY_TURN; }
  uint8_t myIndex()  const { return _myIndex; }

private:
  enum State : uint8_t { WAITING, MY_TURN };

  int8_t  indexOf(uint8_t addr) const;
  uint8_t nextIndex(uint8_t i) const { return (uint8_t)((i + 1) % _ringSize); }
  void    grantTo(uint8_t idx);   // grant ring[idx]; if that's us, take the turn
  void    takeTurn();             // announce our claim and become MY_TURN

  TwrEngine* _engine   = nullptr;
  uint8_t    _myAddr   = UWB_ADDR_INVALID;
  uint8_t    _ring[UWB_RING_MAX];
  uint8_t    _ringSize = 0;
  uint8_t    _myIndex  = 0;

  State    _state    = WAITING;
  uint8_t  _holder   = UWB_ADDR_INVALID;  // who we believe holds the turn (diag)

  // Grantor watchdog: we issued an outstanding grant and are waiting on it.
  bool     _watching     = false;
  uint8_t  _watchIdx     = 0;             // ring index we granted
  bool     _watchClaimed = false;         // did that grantee ANNOUNCE?
  uint32_t _watchSinceMs = 0;             // start of the current watch phase

  uint32_t _lastActivityMs = 0;           // last time ANY ring frame was heard
};

#endif // UWBRTLS_TAGRING_H
