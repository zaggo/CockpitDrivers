#ifndef CAN_H
#define CAN_H
#include <Arduino.h>
#include <BaseCAN.h>
#include "Configuration.h"
#include <MotionMessageId.h>
#include <MotionNodeId.h>
#include "MotionActor.h"

class CAN : public BaseCAN
{
public:
    CAN(MotionActor *motionActor);
    ~CAN();

    bool begin() override;

    void loop();

private:
    bool gatewayAlive = false;

    void handleFrame(MotionMessageId id, uint8_t ext, uint8_t len, const uint8_t *data);
    void onGatewayHeartbeatTimeout();
    void onGatewayHeartbeatDiscovered();

    // Heartbeat monitoring
    uint32_t lastGatewayHeartbeat = 0;
    uint32_t lastActorHeartbeat = 0;
    static const uint32_t HEARTBEAT_INTERVAL = 500; // 0.5 second
    static const uint32_t GATEWAY_TIMEOUT = 1500;   // 1.5 seconds

    void sendActorHeartbeat();
    void updateGatewayHeartbeat(uint8_t len, const uint8_t *data);

    MotionActor *motionActor;
};
#endif // CAN_H