#include "TagRing.h"

void TagRing::begin(TwrEngine* engine, uint8_t myAddr,
                    const uint8_t* ring, uint8_t ringSize) {
  _engine   = engine;
  _myAddr   = myAddr;
  _ringSize = (ringSize > UWB_RING_MAX) ? UWB_RING_MAX : ringSize;
  if (_ringSize == 0) _ringSize = 1;     // degenerate guard (solo free-run)
  for (uint8_t i = 0; i < _ringSize; i++) _ring[i] = ring[i];

  int8_t mi = indexOf(myAddr);
  _myIndex  = (mi >= 0) ? (uint8_t)mi : 0;
  if (mi < 0)
    Serial.printf("[RING] WARNING: addr 0x%02X not in ring — using index 0\n", myAddr);

  _state          = WAITING;
  _watching       = false;
  _watchClaimed   = false;
  _holder         = UWB_ADDR_INVALID;
  _lastActivityMs = millis();            // start the reclaim clock now
  Serial.printf("[RING] begin: addr 0x%02X, index %u of %u\n",
                _myAddr, _myIndex, _ringSize);
}

int8_t TagRing::indexOf(uint8_t addr) const {
  for (uint8_t i = 0; i < _ringSize; i++)
    if (_ring[i] == addr) return (int8_t)i;
  return -1;
}

void TagRing::takeTurn() {
  // Claim the turn: broadcast an ANNOUNCE so our grantor/peers see us within a
  // few ms, then enter MY_TURN. The caller's next poll() returns true → sweep.
  _engine->sendAnnounce(_myAddr);
  _state          = MY_TURN;
  _holder         = _myAddr;
  _watching       = false;
  _lastActivityMs = millis();
  Serial.println("[RING] >> my turn");
}

void TagRing::grantTo(uint8_t idx) {
  if (_ring[idx] == _myAddr) {           // ring wrapped back to us → take it
    takeTurn();
    return;
  }
  _engine->sendSlotGrant(_ring[idx]);
  _holder         = _ring[idx];
  _watching       = true;
  _watchIdx       = idx;
  _watchClaimed   = false;
  _watchSinceMs   = millis();
  _lastActivityMs = _watchSinceMs;       // our own token counts as activity
  Serial.printf("[RING] grant -> 0x%02X\n", _ring[idx]);
}

bool TagRing::poll() {
  if (_state == MY_TURN) return true;    // still our turn (e.g. solo) → sweep
  uint32_t now = millis();

  // 1) Drain and interpret any received frames (tokens, announces, overheard).
  uint8_t type, src, p0;
  while (_engine->pollFrame(type, src, p0)) {
    _lastActivityMs = now;
    if (type == MSG_SLOT_GRANT) {
      if (p0 == _myAddr) {               // granted to us → take the turn
        takeTurn();
        return true;
      }
      _holder   = p0;                    // someone else was granted
      _watching = false;                 // the ring advanced past us
    } else if (type == MSG_ANNOUNCE) {
      if (_watching && _ring[_watchIdx] == p0) {
        _watchClaimed = true;            // our grantee accepted its turn
        _watchSinceMs = now;             // restart timer for the (longer) sweep
      } else {
        _watching = false;               // the ring advanced to someone else
      }
      _holder = p0;
    } else if (src >= UWB_ADDR_TAG_BASE && src != _myAddr &&
               (type == MSG_POLL || type == MSG_RANGE)) {
      // Overheard another TAG actively sweeping: it believes it holds the turn.
      // Without this, two tags that both free-ran (e.g. staggered boot) never
      // resynchronise — grants are only heard while WAITING, but each tag's
      // grant always lands mid-sweep of the other, so both wrap the token back
      // to themselves forever, colliding ("0xF0/0xF1 silent — skip" on both
      // sides while both are in fact ranging). Yield instead of wrapping:
      if (_watching && _ring[_watchIdx] == src) {
        if (!_watchClaimed)
          Serial.printf("[RING] 0x%02X sweeping — implicit claim\n", src);
        _watchClaimed = true;            // grantee is alive (its ANNOUNCE was
        _watchSinceMs = now;             // lost/skipped) — give it the full
                                         // COMPLETE budget to finish + hand off
      } else if (_holder != src) {
        Serial.printf("[RING] 0x%02X owns the channel — standing down\n", src);
        _holder   = src;                 // someone else holds the turn: stand
        _watching = false;               // down and wait for their handoff
      }
    }
    // Anchor-sourced frames (ACK/REPORT) just refresh activity above.
  }

  // 2) Grantor watchdog: advance the token past a slot that didn't take/finish.
  if (_watching) {
    uint32_t budget = _watchClaimed ? UWB_RING_COMPLETE_TIMEOUT_MS
                                    : UWB_RING_CLAIM_TIMEOUT_MS;
    if (now - _watchSinceMs > budget) {
      Serial.printf("[RING] 0x%02X %s — skip\n", _ring[_watchIdx],
                    _watchClaimed ? "stalled" : "silent");
      grantTo(nextIndex(_watchIdx));     // may wrap back to us → takeTurn
      return (_state == MY_TURN);
    }
    return false;
  }

  // 3) Total-silence backstop (cold start / dead ring). Staggered by index so
  //    the lowest live index reclaims first and symmetry is broken.
  uint32_t reclaim = UWB_RING_RECLAIM_BASE_MS +
                     (uint32_t)_myIndex * UWB_RING_RECLAIM_STAGGER_MS;
  if (now - _lastActivityMs > reclaim) {
    Serial.println("[RING] ring silent — reclaim");
    takeTurn();
    return true;
  }

  return false;
}

void TagRing::handoff() {
  _state = WAITING;
  grantTo(nextIndex(_myIndex));          // hand the turn to the next position
}
