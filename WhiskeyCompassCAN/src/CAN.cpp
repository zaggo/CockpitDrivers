#include "CAN.h"
#include "Configuration.h"
#include "DebugLog.h"

CAN::CAN(WhiskeyCompass *compass)
    : InstrumentCAN(kCanCSPin, kCanIntPin, CANFirmwareInfo{static_cast<uint16_t>(kNodeId), 1, 0}),
      compass(compass)
{
    DEBUGLOG_PRINTLN(F("CAN initialized"));
}

void CAN::onStartupFail()
{
    DEBUGLOG_PRINTLN(F("CAN startup FAIL"));
    compass->setBrightness(0);
}

bool CAN::instrumentBegin()
{
    // Beide RX-Buffer vergleichen alle ID-Bits
    canBus->init_Mask(0, 0, MASK_EXACT); // RXB0
    canBus->init_Mask(1, 0, MASK_EXACT); // RXB1

    // RXB0: Compass heading
    canBus->init_Filt(0, 0, CAN_STD_ID(CanMessageId::compass));
    canBus->init_Filt(1, 0, CAN_STD_ID(CanMessageId::compass));

    // RXB1: Lights und Gateway Heartbeat
    canBus->init_Filt(2, 0, CAN_STD_ID(CanMessageId::lights));
    canBus->init_Filt(3, 0, CAN_STD_ID(CanMessageId::gatewayHeartbeat));
    canBus->init_Filt(4, 0, CAN_STD_ID(CanMessageId::lights));
    canBus->init_Filt(5, 0, CAN_STD_ID(CanMessageId::gatewayHeartbeat));

    canBus->setMode(MCP_NORMAL);

    compass->setBrightness(0);

    return true;
}

void CAN::handleFrame(CanMessageId id, uint8_t ext, uint8_t len, const uint8_t *data)
{
    // No String here: heap churn + extra stack in the deepest call path was
    // part of the stack/heap collision that froze the CAN link on other nodes.
    DEBUGLOG_PRINT(F("CAN Message received: ID "));
    DEBUGLOG_PRINTLN(static_cast<uint16_t>(id));

    // We currently expect standard frames only (ext == 0).
    (void)ext;

    switch (id)
    {
    case CanMessageId::compass:
    {
        if (len >= 2)
        {
            // [0..1] magnetic heading, degrees * 100, big-endian.
            const uint16_t deg100 = (static_cast<uint16_t>(data[0]) << 8) |
                                    static_cast<uint16_t>(data[1]);

            // Before homing the card position means nothing, so a setpoint would
            // send it somewhere arbitrary.
            if (compass->isHomed)
            {
                compass->moveToHeading(static_cast<double>(deg100) / 100.);
            }
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
            compass->setBrightness(pwm);
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
    compass->setBrightness(0);
}

void CAN::onGatewayHeartbeatDiscovered()
{
    DEBUGLOG_PRINTLN(F("Gateway heartbeat OK"));
    compass->setBrightness(255);

    // First contact with the gateway is what starts the card: homing before that
    // would drive the instrument with no sim running.
    if (!compass->isHomed && !compass->isHoming())
    {
        compass->beginHoming();
    }
}
