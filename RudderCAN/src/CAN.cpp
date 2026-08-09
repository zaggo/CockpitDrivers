#include "CAN.h"
#include "Configuration.h"
#include "DebugLog.h"

CAN::CAN(Rudder *rudder)
    : InstrumentCAN(kCanCSPin, kCanIntPin, CANFirmwareInfo{static_cast<uint16_t>(kNodeId), 1, 0}),
      rudder(rudder)
{
    DEBUGLOG_PRINTLN(F("CAN initialized"));
}

CAN::~CAN()
{
}

void CAN::loop()
{
    InstrumentCAN::loop();

    // Must run on every pass — the filters need their 500Hz sample rate, which
    // the send gate below would otherwise throttle to 50Hz.
    rudder->sample();

    const uint32_t now = millis();
    if (now - lastSendTime < kMinSendIntervalMs)
    {
        return;
    }

    RudderStateUpdate update = rudder->getStateUpdate();

    // Event-driven: send as soon as an axis moved past its noise threshold,
    // otherwise fall back to the periodic refresh.
    if (!update.changed && (now - lastSendTime < kPeriodicSendIntervalMs))
    {
        return;
    }

    sendRudderState(update.state);
    lastSendTime = now;

    // No String here: heap churn + extra stack in the deepest call path is what
    // froze the CAN link on other AVR nodes.
    DEBUGLOG_PRINT(F("Rudder "));
    DEBUGLOG_PRINT(update.state.rudder);
    DEBUGLOG_PRINT(F(" lBrk "));
    DEBUGLOG_PRINT(update.state.leftBrake);
    DEBUGLOG_PRINT(F(" rBrk "));
    DEBUGLOG_PRINTLN(update.state.rightBrake);
}

void CAN::sendRudderState(const RudderState &state)
{
    // Big endian, matching the rest of the bus. [6..7] stay reserved/zero.
    const uint16_t rudderBits = static_cast<uint16_t>(state.rudder);
    byte payload[8] = {0};
    payload[0] = static_cast<uint8_t>(rudderBits >> 8);
    payload[1] = static_cast<uint8_t>(rudderBits & 0xFF);
    payload[2] = static_cast<uint8_t>(state.leftBrake >> 8);
    payload[3] = static_cast<uint8_t>(state.leftBrake & 0xFF);
    payload[4] = static_cast<uint8_t>(state.rightBrake >> 8);
    payload[5] = static_cast<uint8_t>(state.rightBrake & 0xFF);

    sendMessage(static_cast<uint16_t>(CanMessageId::rudder), 8, payload);
}

void CAN::onStartupFail()
{
    DEBUGLOG_PRINTLN(F("CAN startup FAIL"));
}

bool CAN::instrumentBegin()
{
    // Beide RX-Buffer vergleichen alle ID-Bits
    canBus->init_Mask(0, 0, MASK_EXACT); // RXB0
    canBus->init_Mask(1, 0, MASK_EXACT); // RXB1

    // This node is send-only apart from the gateway heartbeat, so every filter
    // slot is pinned to 0x300 — unused slots must not be left wide open.
    canBus->init_Filt(0, 0, CAN_STD_ID(CanMessageId::gatewayHeartbeat));
    canBus->init_Filt(1, 0, CAN_STD_ID(CanMessageId::gatewayHeartbeat));
    canBus->init_Filt(2, 0, CAN_STD_ID(CanMessageId::gatewayHeartbeat));
    canBus->init_Filt(3, 0, CAN_STD_ID(CanMessageId::gatewayHeartbeat));
    canBus->init_Filt(4, 0, CAN_STD_ID(CanMessageId::gatewayHeartbeat));
    canBus->init_Filt(5, 0, CAN_STD_ID(CanMessageId::gatewayHeartbeat));

    canBus->setMode(MCP_NORMAL);

    return true;
}

void CAN::onGatewayHeartbeatTimeout()
{
    DEBUGLOG_PRINTLN(F("Gateway heartbeat TIMEOUT"));
}

void CAN::onGatewayHeartbeatDiscovered()
{
    DEBUGLOG_PRINTLN(F("Gateway heartbeat OK"));
}

void CAN::handleFrame(CanMessageId id, uint8_t ext, uint8_t len, const uint8_t *data)
{
    // We currently expect standard frames only (ext == 0).
    (void)ext;
    (void)len;
    (void)data;

    // Gateway Heartbeat is already handled by InstrumentCAN base class
    switch (id)
    {
    default:
        DEBUGLOG_PRINT(F("Handle CAN frame Id: "));
        DEBUGLOG_PRINTLN(static_cast<uint16_t>(id));
        break;
    }
}
