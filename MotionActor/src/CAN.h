#ifndef CAN_H
#define CAN_H
#include <Arduino.h>
#include <BaseCAN.h>
#include "Configuration.h"
#include <MotionMessageId.h>
#include <MotionNodeId.h>
#include "MotionActor.h"

#if MOTION_TESTBENCH
class TestBench;
#endif

class CAN : public BaseCAN
{
public:
    CAN(MotionActor *motionActor);
    ~CAN();

    bool begin() override;

    void loop();

#if MOTION_TESTBENCH
    void setTestBench(TestBench *bench) { testBench = bench; }
#endif

private:
#if MOTION_TESTBENCH
    TestBench *testBench = nullptr;
#endif
    bool gatewayAlive = false;
    bool gatewayHeartbeatSeen = false;

    void handleFrame(MotionMessageId id, uint8_t ext, uint8_t len, const uint8_t *data);
    void onGatewayHeartbeatTimeout();
    void onGatewayHeartbeatDiscovered();
    void updateStatusLeds();
    void resetHeartbeatClocks();

    // Latest demand seen while draining RX frames. Applied once per loop() pass so the
    // blocking Kangaroo serial round-trip never runs inside the drain loop (RX overflow
    // there drops gateway heartbeats).
    bool demandPending = false;
    uint16_t pendingDemand1 = 0;
    uint16_t pendingDemand2 = 0;

    // Heartbeat monitoring
    uint32_t lastGatewayHeartbeat = 0;
    uint32_t lastActorHeartbeat = 0;
    uint32_t lastLedBlinkToggle = 0;
    bool ledBlinkStateOn = false;
    static const uint32_t HEARTBEAT_INTERVAL = 500; // 0.5 second
    static const uint32_t GATEWAY_TIMEOUT = 1500;   // 1.5 seconds
    static const uint32_t LED_BLINK_INTERVAL = 300;

    void sendActorHeartbeat();
    void updateGatewayHeartbeat(uint8_t len, const uint8_t *data);

    MotionActor *motionActor;
};
#endif // CAN_H