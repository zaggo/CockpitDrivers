#include "CAN.h"
#include "Configuration.h"
#include "DebugLog.h"

CAN::CAN(RPMGauge *rpmGauge, Odometer *odometer)
    : InstrumentCAN(kCanCSPin, kCanIntPin, CANFirmwareInfo{static_cast<uint16_t>(kNodeId), 1, 0}),
      rpmGauge(rpmGauge),
      odometer(odometer)
{
    DEBUGLOG_PRINTLN(F("CAN initialized"));
}

void CAN::onStartupFail()
{
    DEBUGLOG_PRINTLN(F("CAN startup FAIL"));
    rpmGauge->setBrightness(0);
}

bool CAN::instrumentBegin()
{
    // Beide RX-Buffer vergleichen alle ID-Bits
    canBus->init_Mask(0, 0, MASK_EXACT); // RXB0
    canBus->init_Mask(1, 0, MASK_EXACT); // RXB1

    // RXB0: RPM
    canBus->init_Filt(0, 0, CAN_STD_ID(CanMessageId::rpm));
    canBus->init_Filt(1, 0, CAN_STD_ID(CanMessageId::odometer));

    // RXB1: Lights und Gateway Heartbeat
    canBus->init_Filt(2, 0, CAN_STD_ID(CanMessageId::lights));
    canBus->init_Filt(3, 0, CAN_STD_ID(CanMessageId::gatewayHeartbeat));
    canBus->init_Filt(4, 0, CAN_STD_ID(CanMessageId::lights));
    canBus->init_Filt(5, 0, CAN_STD_ID(CanMessageId::gatewayHeartbeat));

    canBus->setMode(MCP_NORMAL);

    rpmGauge->moveNeedle(0);
    rpmGauge->setBrightness(0);
    float zeroDigits[6] = {0., 0., 0., 0., 0., 0.};
    odometer->displayNumber(zeroDigits);

    return true;
}

void CAN::handleFrame(CanMessageId id, uint8_t ext, uint8_t len, const uint8_t *data)
{
    DEBUGLOG_PRINTLN(String(F("CAN Message received: ID 0x")) + String(static_cast<uint16_t>(id), HEX));

    // We currently expect standard frames only (ext == 0).
    (void)ext;

    // IDs from mcp_can are the actual 11-bit ID (e.g. 0x204), even though filters use (ID<<16).
    switch (id)
    {
    case CanMessageId::rpm:
    {
        if (len >= 2)
        {
            const uint16_t rpm = (static_cast<uint16_t>(data[0]) << 8) | static_cast<uint16_t>(data[1]);
            rpmGauge->moveNeedle(rpm);
        }
        break;
    }

    case CanMessageId::odometer:
    {
        if (len >= 7)
        {
            // Odometer is sent as 5 bytes for Hrs x 1000, x100, x10, x1, x0.1 and 2 bytes for Hrs x0.01 * 1000
            float digits[6];
            digits[0] = static_cast<float>(data[0]);
            digits[1] = static_cast<float>(data[1]);
            digits[2] = static_cast<float>(data[2]);
            digits[3] = static_cast<float>(data[3]);
            digits[4] = static_cast<float>(data[4]);
            digits[5] = static_cast<float>((static_cast<uint16_t>(data[5]) << 8) | static_cast<uint16_t>(data[6])) / 1000.;
            odometer->displayNumber(digits);
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
            rpmGauge->setBrightness(pwm);
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
    rpmGauge->setBrightness(0);
}

void CAN::onGatewayHeartbeatDiscovered()
{
    DEBUGLOG_PRINTLN(F("Gateway heartbeat OK"));
    rpmGauge->setBrightness(255);
}
