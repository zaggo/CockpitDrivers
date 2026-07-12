#ifndef CANIDERROR_H
#define CANIDERROR_H
#include <stdint.h>

// Error types for CAN ID tracking
enum class CanErrorType : uint8_t {
    NONE = 0,
    TX_ERROR = 1,          // Transmission error
    RX_ERROR = 2,          // Reception error
    HEARTBEAT_TIMEOUT = 3  // Heartbeat timeout
};

// Simple struct for tracking CAN ID errors (Arduino doesn't support std::map)
struct CanIdError {
    uint16_t canId;
    bool hasError;
    CanErrorType errorType;
};

inline bool anyCanIdHasError(const CanIdError* errors, uint8_t count) {
    for (uint8_t i = 0; i < count; ++i) {
        if (errors[i].hasError) {
            return true;
        }
    }
    return false;
}

#endif // CANIDERROR_H
