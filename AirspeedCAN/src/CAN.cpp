#include "CAN.h"
#include "Configuration.h"
#include "DebugLog.h"

CAN::CAN(AirspeedIndicator *airspeedIndicator)
    : InstrumentCAN(kCanCSPin, kCanIntPin, CANFirmwareInfo{static_cast<uint16_t>(kNodeId), 1, 0}),
      airspeedIndicator(airspeedIndicator)
{
    DEBUGLOG_PRINTLN(F("CAN initialized"));
}

void CAN::onStartupFail()
{
    DEBUGLOG_PRINTLN(F("CAN startup FAIL"));
    airspeedIndicator->setBrightness(0);
}

bool CAN::instrumentBegin()
{
    // Beide RX-Buffer vergleichen alle ID-Bits
    canBus->init_Mask(0, 0, MASK_EXACT); // RXB0
    canBus->init_Mask(1, 0, MASK_EXACT); // RXB1

    // RXB0: Airspeed
    canBus->init_Filt(0, 0, CAN_STD_ID(CanMessageId::airspeed));
    canBus->init_Filt(1, 0, CAN_STD_ID(CanMessageId::airspeed));

    // RXB1: Lights und Gateway Heartbeat
    canBus->init_Filt(2, 0, CAN_STD_ID(CanMessageId::lights));
    canBus->init_Filt(3, 0, CAN_STD_ID(CanMessageId::gatewayHeartbeat));
    canBus->init_Filt(4, 0, CAN_STD_ID(CanMessageId::lights));
    canBus->init_Filt(5, 0, CAN_STD_ID(CanMessageId::gatewayHeartbeat));

    canBus->setMode(MCP_NORMAL);

    airspeedIndicator->moveNeedle(0.);
    airspeedIndicator->setBrightness(0);

    return true;
}

void CAN::handleFrame(CanMessageId id, uint8_t ext, uint8_t len, const uint8_t *data)
{
    // No String here: heap churn + extra stack in the deepest call path was
    // part of the stack/heap collision that froze the CAN link.
    DEBUGLOG_PRINT(F("CAN Message received: ID "));
    DEBUGLOG_PRINTLN(static_cast<uint16_t>(id));

    // We currently expect standard frames only (ext == 0).
    (void)ext;

    // IDs from mcp_can are the actual 11-bit ID (e.g. 0x100), even though filters use (ID<<16).
    switch (id)
    {
    case CanMessageId::airspeed:
    {
        if (len >= 2)
        {
            // [0..1] IAS in knots * 10. Bytes 2..3 carry TAS, which this board
            // ignores — the TAS ring is mechanical and has no motor.
            const uint16_t iasTimes10 = (static_cast<uint16_t>(data[0]) << 8) | static_cast<uint16_t>(data[1]);
            airspeedIndicator->moveNeedle(static_cast<float>(iasTimes10) / 10.);
        }
        break;
    }

    case CanMessageId::lights:
    {
        if (len >= 8)
        {
            const uint16_t panelDim1000 = (static_cast<uint16_t>(data[0]) << 8) | static_cast<uint16_t>(data[1]);
            float ratio = constrain(static_cast<float>(panelDim1000) / 1000., 0., 1.);
            uint8_t pwm = static_cast<uint8_t>(ratio * 255.);
            airspeedIndicator->setBrightness(pwm);
        }
        break;
    }

    default:
        break;
    }
}

void CAN::onGatewayHeartbeatTimeout()
{
    DEBUGLOG_PRINTLN(F("Gateway heartbeat TIMEOUT"));
    airspeedIndicator->setBrightness(0);
}

void CAN::onGatewayHeartbeatDiscovered()
{
    DEBUGLOG_PRINTLN(F("Gateway heartbeat OK"));
    airspeedIndicator->setBrightness(255);
}
