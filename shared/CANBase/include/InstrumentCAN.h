#ifndef INSTRUMENTCAN_H
#define INSTRUMENTCAN_H

#include "BaseCAN.h"
#include "CanMessageId.h"
#include "CanNodeId.h"

class InstrumentCAN : public BaseCAN
{
public:
    InstrumentCAN(uint8_t csPin, uint8_t intPin, CANFirmwareInfo fwInfo);
    virtual ~InstrumentCAN();

    // Start CAN and setup filters
    bool begin() override;

    void loop();

protected:
    virtual bool instrumentBegin() = 0;

    // Override this to handle incoming CAN messages
    virtual void handleFrame(CanMessageId id, uint8_t ext, uint8_t len, const uint8_t* data) = 0;

    virtual void onStartupFail() {}
    virtual void onGatewayHeartbeatTimeout() {}
    virtual void onGatewayHeartbeatDiscovered() {}

    bool gatewayAlive = false;

private:
    // Heartbeat monitoring
    uint32_t lastGatewayHeartbeat = 0;
    uint32_t lastInstrumentHeartbeat = 0;
    static const uint32_t HEARTBEAT_INTERVAL = 500;  // 0.5 second
    static const uint32_t GATEWAY_TIMEOUT = 1500;     // 1.5 seconds

    void sendInstrumentHeartbeat();
    void updateGatewayHeartbeat(uint8_t len, const uint8_t* data);
};

#endif // INSTRUMENTCAN_H
