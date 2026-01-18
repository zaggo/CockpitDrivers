#pragma once

#include <cstdint>
#include <vector>
#include <optional>

enum class MessageType : uint8_t {
    Fuel   = 0x01,
    Lights = 0x02,
    // TODO: Add more types based on CAN Message IDs
};

struct Message {
    MessageType type;
    std::vector<uint8_t> payload;
};

class TransportLayer {
public:
    /// Encodes a message into a framed format.
    /// Frame: AA 55 | type(u8) | len(u8) | payload[len]
    /// 
    /// @param type Message type
    /// @param payload Pointer to payload data (can be nullptr if len == 0)
    /// @param len Payload length (max 255)
    /// @return Encoded frame, or empty vector on error (payload too large)
    static std::vector<uint8_t> encodeFrame(MessageType type, const void* payload, uint8_t len);
    
    /// Decodes a message from a buffer.
    /// Expects frame format: AA 55 | type(u8) | len(u8) | payload[len]
    /// 
    /// @param buffer Input buffer
    /// @param len Buffer length (must be at least 4 + payloadLen)
    /// @return Decoded message, or std::nullopt if frame is invalid/incomplete
    static std::optional<Message> decodeFrame(const uint8_t* buffer, size_t len);
    
private:
    static constexpr uint8_t FRAME_SYNC_0 = 0xAA;
    static constexpr uint8_t FRAME_SYNC_1 = 0x55;
    static constexpr size_t MAX_PAYLOAD = 255;
};