#ifndef CAN_H
#define CAN_H
#include <Arduino.h>
#include <BaseCAN.h>
#include "Configuration.h"
#include <MotionMessageId.h>
#include <MotionNodeId.h>

// Forward declaration
class MotionSender;

// Error types for CAN ID tracking
enum class CanErrorType : uint8_t {
    NONE = 0,
    TX_ERROR = 1,      // Transmission error
    RX_ERROR = 2,      // Reception error
    HEARTBEAT_TIMEOUT = 3  // Heartbeat timeout
};

// Simple struct for tracking CAN ID errors (Arduino doesn't support std::map)
struct CanIdError {
    uint16_t canId;
    bool hasError;
    CanErrorType errorType;
};

class CAN : public BaseCAN {
    public:
        CAN();
        ~CAN();

        bool begin() override;

        void loop();

        void sendMessage(MotionMessageId id, uint8_t len, byte* data);
        
        void setMotionSender(MotionSender* sender);

    private:
        uint32_t lastGatewayHeartbeatSendMs = 0;

        // Actor heartbeat monitoring (nodeId -> last seen)
        static constexpr uint8_t kMaxActorNodes = 16; // 0..15
        uint32_t lastActorHeartbeatMs[kMaxActorNodes] = {0};
        bool actorAlive[kMaxActorNodes] = {false};

        // CAN ID error tracking: tracks TX/RX error status per CAN ID
        static constexpr uint8_t kMaxCanIdErrors = 8;
        CanIdError canIdErrors[kMaxCanIdErrors];
        uint8_t canIdErrorCount = 0;

        // Reference to MotionSender for sending messages back to MotionGateway Plugin
        MotionSender* motionSender = nullptr;

        // Handle incoming Serial Message frames
        void updateActorHeartbeat(uint8_t len, const uint8_t* data);
        void updateTransponder(uint8_t len, const uint8_t* data);

        void sendGatewayHeartbeat();
        void checkActorHeartbeats();
        void updateAlarmLED();
        void clearCanIdError(uint16_t canId, CanErrorType errorType = CanErrorType::NONE);
        void setCanIdError(uint16_t canId, CanErrorType errorType);

        void handleFrame(uint32_t id, uint8_t ext, uint8_t len, const uint8_t* data);
};

#endif