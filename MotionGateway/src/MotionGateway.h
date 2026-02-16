#ifndef MOTIONGATEWAY_H
#define MOTIONGATEWAY_H
#include <Arduino.h>
#include "Configuration.h"
#include "CAN.h"
#include <SerialMessageId.h>

// Message metadata for maxAge resync
struct MessageMeta {
    unsigned long lastSendTimestamp;
    unsigned long maxAgeMs;
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
        void handleSimFrame(const uint8_t *data);

        void checkMaxAgeResync();

        bool readBytes(uint8_t* dst, size_t n);

        void sendActorPairDemand(MotionNodeId nodeId, uint16_t act1Demand, uint16_t act2Demand);

        void sendHome();
        void sendStop();

        MotionMode mode = MotionMode::mode0;

        // RX state machine
        // Fuel Gauge
        uint32_t actorDemand[kActorNodeCount] = {0}; // Indexed by nodeId (0..5)

        // Message metadata for maxAge resync
        MessageMeta actorDemandMeta[kActorNodeCount] = {0}; // Indexed by nodeId (0..5)
        
        // Reference to CAN bus
        CAN* canBus;
};
#endif // MOTIONGATEWAY_H