#pragma once
#include <cstdint>
#include <cstddef>

// Encodes six 16-bit actuator setpoints (BFF actuator order) into the frame
// MotionGateway::handleBFFFrame decodes: "BC" reserved MSB[6] LSB[6] CR.
namespace BffEncoder {
    constexpr std::size_t kFrameSize = 16;
    void encode(const uint16_t setpoints[6], uint8_t out[kFrameSize]);
}
