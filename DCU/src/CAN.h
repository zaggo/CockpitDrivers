#ifndef CAN_H
#define CAN_H
#include <Arduino.h>
#include <mcp_can.h>
#include <SPI.h>
#include "Configuration.h"

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

class CAN {
    public:
        CAN();
        ~CAN();

        bool begin();

        void loop();
        void sendMessage(CanStateId id, uint8_t len, byte* data);

    private:
        MCP_CAN* canBus;
        volatile bool isStarted = false;

        // Heartbeat (Variante 2)
        static constexpr uint8_t kNodeId = 0; // DCU/Gateway
        static constexpr uint8_t kFwMajor = 1;
        static constexpr uint8_t kFwMinor = 0;

        uint32_t lastGatewayHeartbeatSendMs = 0;

        // Instrument heartbeat monitoring (nodeId -> last seen)
        static constexpr uint8_t kMaxInstrumentNodes = 16; // 0..15
        uint32_t lastInstrumentHeartbeatMs[kMaxInstrumentNodes] = {0};
        bool instrumentAlive[kMaxInstrumentNodes] = {false};

        // CAN ID error tracking: tracks TX/RX error status per CAN ID
        static constexpr uint8_t kMaxCanIdErrors = 8;
        CanIdError canIdErrors[kMaxCanIdErrors];
        uint8_t canIdErrorCount = 0;

        void sendGatewayHeartbeat();
        void updateInstrumentHeartbeat(uint8_t len, const uint8_t* data);
        void checkInstrumentHeartbeats();
        void updateAlarmLED();
        void clearCanIdError(uint16_t canId, CanErrorType errorType = CanErrorType::NONE);
        void setCanIdError(uint16_t canId, CanErrorType errorType);

        // MCP2515 /INT is active-low and stays low while RX buffers have pending frames.
        // Keep ISR minimal: just set a flag. (Do NOT touch SPI or Serial in ISR.)
        static void onCanInterrupt();
        static volatile bool canIrq;
        static CAN* instance;

        void handleFrame(uint32_t id, uint8_t ext, uint8_t len, const uint8_t* data);
};

#endif