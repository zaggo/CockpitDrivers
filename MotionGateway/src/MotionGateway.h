#ifndef MOTIONGATEWAY_H
#define MOTIONGATEWAY_H
#include <Arduino.h>
#include "Configuration.h"
#include "CAN.h"
#include "MotionSender.h"
#include <SerialMessageId.h>

// Message metadata for maxAge resync
struct MessageMeta {
    unsigned long lastSendTimestamp;
    unsigned long maxAgeMs;
};

class MotionGateway {
    public:
        MotionGateway(CAN* canBus);
        ~MotionGateway();

        void loop();

    private:        
        void handleFrame(const uint8_t *data);
        void checkMaxAgeResync();

        bool readBytes(uint8_t* dst, size_t n);

        void sendActorPairDemand(MotionNodeId nodeId, uint16_t act1Demand, uint16_t act2Demand);

        // RX state machine
        // Fuel Gauge
        uint32_t actorDemand[kActorNodeCount] = {0}; // Indexed by nodeId (0..5)

        // Message metadata for maxAge resync
        MessageMeta actorDemandMeta[kActorNodeCount] = {0}; // Indexed by nodeId (0..5)
        
        // Reference to CAN bus
        CAN* canBus;
        
        // MotionSender instance for sending data back to MotionGateway Plugin
        MotionSender* motionSender;
};
#endif // MOTIONGATEWAY_H