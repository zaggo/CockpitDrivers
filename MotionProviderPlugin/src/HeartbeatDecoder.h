#pragma once
#include <cstdint>

// Decodes the MotionGateway arm heartbeat: 'H' 'B' <armed 0x00|0x01> CR.
// Pure byte-in/frame-out state machine, no threads or I/O — unit-testable.
class HeartbeatDecoder {
public:
    // Feed one received byte. Returns true exactly on the byte that completes a
    // valid frame; call armed() afterwards to read the payload.
    bool feed(uint8_t b);

    // Last decoded armed value (meaningful only right after feed() returns true).
    bool armed() const { return armed_; }

    // Discard any in-progress frame (e.g. on reconnect).
    void reset();

private:
    enum class S : uint8_t { SyncH, SyncB, Payload, CR };
    S       state_   = S::SyncH;
    uint8_t pending_ = 0;
    bool    armed_   = false;
};
