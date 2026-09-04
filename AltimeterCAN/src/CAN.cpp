#include "CAN.h"
#include "Configuration.h"
#include "DebugLog.h"

CAN::CAN(Altimeter *altimeter)
    : InstrumentCAN(kCanCSPin, kCanIntPin, CANFirmwareInfo{static_cast<uint16_t>(kNodeId), 1, 0}),
      altimeter(altimeter)
{
    DEBUGLOG_PRINTLN(F("CAN initialized"));
}

void CAN::onStartupFail()
{
    DEBUGLOG_PRINTLN(F("CAN startup FAIL"));
    altimeter->setBrightness(0);
}

bool CAN::instrumentBegin()
{
    // Beide RX-Buffer vergleichen alle ID-Bits
    canBus->init_Mask(0, 0, MASK_EXACT); // RXB0
    canBus->init_Mask(1, 0, MASK_EXACT); // RXB1

    // RXB0: Altimeter/VSI
    canBus->init_Filt(0, 0, CAN_STD_ID(CanMessageId::altimeterVsi));
    canBus->init_Filt(1, 0, CAN_STD_ID(CanMessageId::altimeterVsi));

    // RXB1: Lights und Gateway Heartbeat
    canBus->init_Filt(2, 0, CAN_STD_ID(CanMessageId::lights));
    canBus->init_Filt(3, 0, CAN_STD_ID(CanMessageId::gatewayHeartbeat));
    canBus->init_Filt(4, 0, CAN_STD_ID(CanMessageId::lights));
    canBus->init_Filt(5, 0, CAN_STD_ID(CanMessageId::gatewayHeartbeat));

    canBus->setMode(MCP_NORMAL);

    altimeter->setBrightness(0);

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
    case CanMessageId::altimeterVsi:
    {
        if (len >= 4)
        {
            // [0..3] altitude in feet, signed. Bytes 4..5 carry the VSI, which
            // this board ignores — that is VerticalSpeedCAN's half of the frame.
            const int32_t altitudeFt = static_cast<int32_t>(
                (static_cast<uint32_t>(data[0]) << 24) |
                (static_cast<uint32_t>(data[1]) << 16) |
                (static_cast<uint32_t>(data[2]) << 8) |
                static_cast<uint32_t>(data[3]));

            // Before homing the axis positions mean nothing, so a setpoint would
            // send the needles somewhere arbitrary.
            if (altimeter->isHomed)
            {
                altimeter->moveToHeight(static_cast<double>(altitudeFt));
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
            altimeter->setBrightness(pwm);
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
    altimeter->setBrightness(0);
}

void CAN::onGatewayHeartbeatDiscovered()
{
    DEBUGLOG_PRINTLN(F("Gateway heartbeat OK"));
    altimeter->setBrightness(255);
}
