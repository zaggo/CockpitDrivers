#ifndef HEARTBEAT_H
#define HEARTBEAT_H
#include <stdint.h>

// True if a heartbeat/value was seen at least once and is within timeoutMs of now.
// Unsigned subtraction makes this correct across a millis() rollover.
inline bool heartbeatAlive(uint32_t lastSeenMs, uint32_t nowMs, uint32_t timeoutMs) {
    return (lastSeenMs != 0) && (nowMs - lastSeenMs <= timeoutMs);
}

// True if a value was sent at least once and is older than maxAgeMs (due for resync).
inline bool isStale(uint32_t lastSendMs, uint32_t nowMs, uint32_t maxAgeMs) {
    return (lastSendMs > 0) && (nowMs - lastSendMs >= maxAgeMs);
}

#endif // HEARTBEAT_H
