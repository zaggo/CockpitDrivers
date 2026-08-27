#ifndef MOTIONGATEWAY_H
#define MOTIONGATEWAY_H
#include <Arduino.h>
#include "Configuration.h"
#include "CAN.h"
#include "GatewayStats.h"
#include <SerialMessageId.h>

// Message metadata for maxAge resync
struct MessageMeta {
    unsigned long lastSendTimestamp;
    unsigned long maxAgeMs;
};

// Actor mapping: defines which actor input maps to which node and motor
struct ActorMapping {
    MotionNodeId nodeId; // Target actor pair
    uint8_t motorIndex;  // 0 or 1: Motor in the pair (0=m1, 1=m2)
};

enum class MotionMode: uint8_t {
    mode0 = 0, // Off
    mode1 = 1, // BFF Motion Driver compatible mode (0-100% mapped to 0-65280 demand range)
    mode2 = 2  // Sim Mode
};

class MotionGateway {
    public:
        MotionGateway(CAN* canBus);
        ~MotionGateway();

        void loop();

    private:
        void handleSerialInput();
        void handleBFFFrame(const uint8_t *data);
        void handleSimToolsFrame(const uint8_t *data);
        void handleGotoFrame(const uint8_t *data);
        void processGoto();
        void sendActorPairGoto(MotionNodeId nodeId, uint16_t act1Target,
                               uint16_t act2Target, uint16_t durationMs);

        void processDemands(const uint16_t demand[6]);

        void checkMaxAgeResync();

        bool readBytes(uint8_t* dst, size_t n);

        void sendActorPairDemand(MotionNodeId nodeId, uint16_t act1Demand, uint16_t act2Demand);

        void sendHome();
        void sendStop();
        void sendUsbHeartbeat();

        MotionMode mode = MotionMode::mode0;
        unsigned long lastModeCheckTimestampMs = 0;
        unsigned long lastDemandBatchSendTimestampMs = 0;
        unsigned long lastUsbHeartbeatTimestampMs = 0;
        unsigned long lastStatsPrintMs = 0;

        // Newest complete frame from the serial drain, applied once per loop()
        // AFTER the drain - the blocking CAN sends must not run inside the RX
        // loop (they stall it long enough to overflow the serial buffer).
        uint16_t pendingDemand[6] = {0};
        bool pendingDemandValid = false;

        // Newest complete goto frame, applied once per loop() after the drain
        // (same rule as pendingDemand). A goto supersedes a demand parsed in
        // the same drain pass.
        uint16_t pendingGoto[6] = {0};
        uint16_t pendingGotoDurationMs = 0;
        bool pendingGotoValid = false;

        GatewayStats stats;
        int8_t lastArmedState = -1; // -1 = unknown; logs a DEBUGLOG line on each change

        // Actor mappings for different modes (6 actors)
        static const ActorMapping actorMappingMode1[6];
        static const ActorMapping actorMappingMode2[6];

        // RX state machine
        uint32_t actorDemand[kActorNodeCount] = {0}; // Indexed by nodeId 

        // Message metadata for maxAge resync
        MessageMeta actorDemandMeta[kActorNodeCount] = {0}; // Indexed by nodeId
        
        // Reference to CAN bus
        CAN* canBus;
};
#endif // MOTIONGATEWAY_H