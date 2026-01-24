#include "TransportLayer.h"
#include <cstring>

std::vector<uint8_t> TransportLayer::encodeFrame(MessageType type, const void* payload, uint8_t len) {
    std::vector<uint8_t> frame;
    
    // Validate payload length
    if (len > MAX_PAYLOAD) {
        return frame;  // Return empty on error
    }
    
    // Frame: AA 55 | type(u8) | len(u8) | payload[len]
    frame.reserve(4 + len);
    
    // Sync bytes
    frame.push_back(FRAME_SYNC_0);  // 0xAA
    frame.push_back(FRAME_SYNC_1);  // 0x55
    
    // Message type
    frame.push_back(static_cast<uint8_t>(type));
    
    // Payload length
    frame.push_back(len);
    
    // Payload
    if (payload && len > 0) {
        const uint8_t* data = static_cast<const uint8_t*>(payload);
        frame.insert(frame.end(), data, data + len);
    }
    
    return frame;
}

std::optional<Message> TransportLayer::decodeFrame(const uint8_t* buffer, size_t len) {
    // Minimum frame size: 4 bytes (AA 55 | type | len) + at least 0 payload
    if (!buffer || len < 4) {
        return std::nullopt;
    }
    
    // Check sync bytes
    if (buffer[0] != FRAME_SYNC_0 || buffer[1] != FRAME_SYNC_1) {
        return std::nullopt;
    }
    
    // Extract type and payload length
    MessageType type = static_cast<MessageType>(buffer[2]);
    uint8_t payloadLen = buffer[3];
    
    // Validate: we need at least 4 + payloadLen bytes
    if (len < static_cast<size_t>(4 + payloadLen)) {
        return std::nullopt;  // Incomplete frame
    }
    
    // Validate payload length
    if (payloadLen > MAX_PAYLOAD) {
        return std::nullopt;
    }
    
    // Extract payload
    Message msg;
    msg.type = type;
    msg.payload.resize(payloadLen);
    
    if (payloadLen > 0) {
        std::memcpy(msg.payload.data(), buffer + 4, payloadLen);
    }
    
    return msg;
}