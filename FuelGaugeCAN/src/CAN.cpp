#include "CAN.h"
#include "Configuration.h"
#include "DebugLog.h"

CAN::CAN(FuelGauge *fuelGauge) 
    : InstrumentCAN(kCanCSPin, kCanIntPin,{CanNodeId::fuelGaugeNodeId, 1, 0}),
      fuelGauge(fuelGauge)
{
    DEBUGLOG_PRINTLN(F("CAN initialized"));
}

void CAN::onStartupFail()
{
    DEBUGLOG_PRINTLN(F("CAN startup FAIL"));
    fuelGauge->setBrightness(0);
}

bool CAN::instrumentBegin()
{
    // Beide RX-Buffer vergleichen alle ID-Bits
    canBus->init_Mask(0, 0, MASK_EXACT); // RXB0
    canBus->init_Mask(1, 0, MASK_EXACT); // RXB1

    // RXB0: ID 0x270
    canBus->init_Filt(0, 0, CAN_STD_ID(CanMessageId::fuelLevel));
    canBus->init_Filt(1, 0, CAN_STD_ID(CanMessageId::fuelLevel)); // zweiter Filter optional identisch

    // RXB1: Lights (0x203) und Gateway Heartbeat (0x300)
    canBus->init_Filt(2, 0, CAN_STD_ID(CanMessageId::lights));
    canBus->init_Filt(3, 0, CAN_STD_ID(CanMessageId::gatewayHeartbeat));
    // Optional: weitere Filter-Slots frei lassen / duplizieren (je nach MCP2515-Lib erforderlich)
    canBus->init_Filt(4, 0, CAN_STD_ID(CanMessageId::lights));
    canBus->init_Filt(5, 0, CAN_STD_ID(CanMessageId::gatewayHeartbeat));

    canBus->setMode(MCP_NORMAL);

    fuelGauge->moveServo(leftTank, 0);
    fuelGauge->moveServo(rightTank, 0);
    fuelGauge->setBrightness(0);

    return true;
}

void CAN::handleFrame(CanMessageId id, uint8_t ext, uint8_t len, const uint8_t *data)
{
    DEBUGLOG_PRINTLN(String(F("CAN Message received: ID 0x")) + String(static_cast<uint16_t>(id), HEX));

    // We currently expect standard frames only (ext == 0).
    (void)ext;

    // IDs from mcp_can are the actual 11-bit ID (e.g. 0x270), even though filters use (ID<<16).
    switch (id)
    {
    case CanMessageId::fuelLevel:
    {
        if (len >= 8)
        {
            const uint16_t fuelLeftKg100 = (static_cast<uint16_t>(data[0]) << 8) | static_cast<uint16_t>(data[1]);
            const uint16_t fuelRightKg100 = (static_cast<uint16_t>(data[2]) << 8) | static_cast<uint16_t>(data[3]);

            fuelGauge->moveServo(leftTank, static_cast<float>(fuelLeftKg100) / 100.);
            fuelGauge->moveServo(rightTank, static_cast<float>(fuelRightKg100) / 100.);
        }
        break;
    }

    case CanMessageId::lights:
    {
        if (len >= 8)
        {
            const uint16_t panelDim1000 = (static_cast<uint16_t>(data[0]) << 8) | static_cast<uint16_t>(data[1]);
            // Not in this instrument:
            // const uint16_t radioDim1000 = (static_cast<uint16_t>(data[2]) << 8) | static_cast<uint16_t>(data[3]);
            // const uint16_t domeLightDim1000 = (static_cast<uint16_t>(data[4]) << 8) | static_cast<uint16_t>(data[5]);
            float ratio = constrain(static_cast<float>(panelDim1000) / 1000., 0., 1.);
            uint8_t pwm = static_cast<uint8_t>(ratio * 255.);
            fuelGauge->setBrightness(pwm);
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
    fuelGauge->setBrightness(0);
}

void CAN::onGatewayHeartbeatDiscovered()
{
    DEBUGLOG_PRINTLN(F("Gateway heartbeat OK"));
    fuelGauge->setBrightness(255);
}