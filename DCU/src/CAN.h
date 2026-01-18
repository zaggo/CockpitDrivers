#ifndef CAN_H
#define CAN_H
#include <Arduino.h>
#include <mcp_can.h>
#include <SPI.h>
#include "Configuration.h"

class CAN {
    public:
        CAN();
        ~CAN();

        bool begin();

        void loop();
        void sendMessage(CanStateId id, uint8_t len, byte* data);

    private:
        MCP_CAN* canBus;
        bool isStarted = false;

        // Heartbeat (Variante 2)
        static constexpr uint8_t kNodeId = 0; // DCU/Gateway
        static constexpr uint8_t kFwMajor = 1;
        static constexpr uint8_t kFwMinor = 0;

        uint32_t lastGatewayHeartbeatSendMs = 0;

        // Instrument heartbeat monitoring (nodeId -> last seen)
        static constexpr uint8_t kMaxInstrumentNodes = 16; // 0..15
        uint32_t lastInstrumentHeartbeatMs[kMaxInstrumentNodes] = {0};
        bool instrumentAlive[kMaxInstrumentNodes] = {false};

        void sendGatewayHeartbeat();
        void updateInstrumentHeartbeat(uint8_t len, const uint8_t* data);
        void checkInstrumentHeartbeats();

        // MCP2515 /INT is active-low and stays low while RX buffers have pending frames.
        // Keep ISR minimal: just set a flag. (Do NOT touch SPI or Serial in ISR.)
        static void onCanInterrupt();
        static volatile bool canIrq;
        static CAN* instance;

        void handleFrame(uint32_t id, uint8_t ext, uint8_t len, const uint8_t* data);
};

#endif