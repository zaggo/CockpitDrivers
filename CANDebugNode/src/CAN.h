#ifndef CAN_H
#define CAN_H
#include <Arduino.h>
#include <mcp_can.h>
#include <SPI.h>
#include "LCD.h"
#include "Configuration.h"

class CAN {
    public:
        CAN(LCD* lcd);
        ~CAN();

        bool begin();
        void loop();
        void sendMessage(CanStateId id, uint8_t len, byte* data);

    private:
        MCP_CAN* canBus;
        LCD* lcd;
        volatile bool isStarted = false;

        // Heartbeat (Variante 2)
        static constexpr uint8_t kNodeId = 1; // FuelGauge
        static constexpr uint8_t kFwMajor = 1;
        static constexpr uint8_t kFwMinor = 0;

        uint32_t lastGatewayHeartbeatMs = 0;
        uint32_t lastInstrumentHeartbeatSendMs = 0;
        bool gatewayAlive = false;

        void sendInstrumentHeartbeat();
        void updateGatewayHeartbeat(uint8_t len, const uint8_t* data);

        // MCP2515 /INT is active-low and stays low while RX buffers have pending frames.
        // Keep ISR minimal: just set a flag. (Do NOT touch SPI or Serial in ISR.)
        static void onCanInterrupt();
        static volatile bool canIrq;
        static CAN* instance;

        void handleFrame(uint32_t id, uint8_t ext, uint8_t len, const uint8_t* data);
};

#endif