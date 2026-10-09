#include "CAN.h"
#include "Configuration.h"
#include "DebugLog.h"

CAN::CAN(AttitudeIndicator *attitude)
    : InstrumentCAN(kCanCSPin, kCanIntPin, CANFirmwareInfo{static_cast<uint16_t>(kNodeId), 1, 0}),
      attitude(attitude)
{
    DEBUGLOG_PRINTLN(F("CAN initialized"));
}

void CAN::onStartupFail()
{
    DEBUGLOG_PRINTLN(F("CAN startup FAIL"));
    attitude->setBrightness(0);
}

bool CAN::instrumentBegin()
{
    // Beide RX-Buffer vergleichen alle ID-Bits
    canBus->init_Mask(0, 0, MASK_EXACT); // RXB0
    canBus->init_Mask(1, 0, MASK_EXACT); // RXB1

    // RXB0: Attitude setpoint
    canBus->init_Filt(0, 0, CAN_STD_ID(CanMessageId::attitude));
    canBus->init_Filt(1, 0, CAN_STD_ID(CanMessageId::attitude));

    // RXB1: Lights und Gateway Heartbeat
    canBus->init_Filt(2, 0, CAN_STD_ID(CanMessageId::lights));
    canBus->init_Filt(3, 0, CAN_STD_ID(CanMessageId::gatewayHeartbeat));
    canBus->init_Filt(4, 0, CAN_STD_ID(CanMessageId::lights));
    canBus->init_Filt(5, 0, CAN_STD_ID(CanMessageId::gatewayHeartbeat));

    canBus->setMode(MCP_NORMAL);

    attitude->setBrightness(0);

    return true;
}

void CAN::handleFrame(CanMessageId id, uint8_t ext, uint8_t len, const uint8_t *data)
{
    // Deliberately no per-frame log here. The DCU re-sends the attitude on every
    // change, so a print per frame is a continuous stream; at 115200 baud
    // Serial.print blocks once the 64-byte TX buffer fills, inside the same
    // loop() that steps the motors.

    // We currently expect standard frames only (ext == 0).
    (void)ext;

    switch (id)
    {
    case CanMessageId::attitude:
    {
        if (len >= 4)
        {
            // [0..1] pitch, [2..3] roll, both int16 degrees * 100, big-endian.
            const int16_t pitch100 = static_cast<int16_t>((static_cast<uint16_t>(data[0]) << 8) |
                                                          static_cast<uint16_t>(data[1]));
            const int16_t roll100 = static_cast<int16_t>((static_cast<uint16_t>(data[2]) << 8) |
                                                         static_cast<uint16_t>(data[3]));
            // Before homing the positions mean nothing, so a setpoint would send
            // the display somewhere arbitrary - possibly into a stop.
            if (attitude->isHomed)
            {
                attitude->moveToAttitude(static_cast<double>(pitch100) / 100.,
                                         static_cast<double>(roll100) / 100.);
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
            panelPwm = static_cast<uint8_t>(ratio * 255.);
            panelPwmKnown = true;
            attitude->setBrightness(panelPwm);
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
    attitude->setBrightness(0);
}

void CAN::onGatewayHeartbeatDiscovered()
{
    DEBUGLOG_PRINTLN(F("Gateway heartbeat OK"));

    // First contact with no lights frame yet: come up lit so the instrument
    // visibly joins the bus. After that, honour whatever we were last told,
    // including zero.
    attitude->setBrightness(panelPwmKnown ? panelPwm : 255);

    // First contact with the gateway is what starts the axes: homing before
    // that would drive the instrument with no sim running.
    if (!attitude->isHomed && !attitude->isHoming())
    {
        attitude->beginHoming();
    }
}
