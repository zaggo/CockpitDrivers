#include "CAN.h"
#include "Configuration.h"
#include "DebugLog.h"

CAN::CAN(TurnCoordinator *turnCoordinator)
    : InstrumentCAN(kCanCSPin, kCanIntPin, CANFirmwareInfo{static_cast<uint16_t>(kNodeId), 1, 0}),
      turnCoordinator(turnCoordinator)
{
    DEBUGLOG_PRINTLN(F("CAN initialized"));
}

void CAN::onStartupFail()
{
    DEBUGLOG_PRINTLN(F("CAN startup FAIL"));
    turnCoordinator->setBrightness(0);
}

bool CAN::instrumentBegin()
{
    // Beide RX-Buffer vergleichen alle ID-Bits
    canBus->init_Mask(0, 0, MASK_EXACT); // RXB0
    canBus->init_Mask(1, 0, MASK_EXACT); // RXB1

    // RXB0: Turn coordinator
    canBus->init_Filt(0, 0, CAN_STD_ID(CanMessageId::turnCoordinator));
    canBus->init_Filt(1, 0, CAN_STD_ID(CanMessageId::turnCoordinator));

    // RXB1: Lights und Gateway Heartbeat
    canBus->init_Filt(2, 0, CAN_STD_ID(CanMessageId::lights));
    canBus->init_Filt(3, 0, CAN_STD_ID(CanMessageId::gatewayHeartbeat));
    canBus->init_Filt(4, 0, CAN_STD_ID(CanMessageId::lights));
    canBus->init_Filt(5, 0, CAN_STD_ID(CanMessageId::gatewayHeartbeat));

    canBus->setMode(MCP_NORMAL);

    turnCoordinator->setTurnCoordinator(0., 0.);
    turnCoordinator->setBrightness(0);

    return true;
}

void CAN::handleFrame(CanMessageId id, uint8_t ext, uint8_t len, const uint8_t *data)
{
    // No per-frame logging: Serial.print blocks long enough to stretch the
    // step period of both motors.

    // We currently expect standard frames only (ext == 0).
    (void)ext;

    switch (id)
    {
    case CanMessageId::turnCoordinator:
    {
        if (len >= 4)
        {
            // [0..1] turn rate, [2..3] slip, both int16 degrees * 100, signed.
            const int16_t turn100 = static_cast<int16_t>((static_cast<uint16_t>(data[0]) << 8) | static_cast<uint16_t>(data[1]));
            const int16_t slip100 = static_cast<int16_t>((static_cast<uint16_t>(data[2]) << 8) | static_cast<uint16_t>(data[3]));
            turnCoordinator->setTurnCoordinator(static_cast<float>(turn100) / 100., static_cast<float>(slip100) / 100.);
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
            turnCoordinator->setBrightness(pwm);
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
    turnCoordinator->setBrightness(0);
}

void CAN::onGatewayHeartbeatDiscovered()
{
    DEBUGLOG_PRINTLN(F("Gateway heartbeat OK"));
    turnCoordinator->setBrightness(255);
}
