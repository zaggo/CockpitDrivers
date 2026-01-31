#include "CAN.h"
#include "Configuration.h"
#include "DebugLog.h"

CAN::CAN(Transponder *transponder)
    : InstrumentCAN(kCanCSPin, kCanIntPin, {CanNodeId::transponderNodeId, 1, 0}),
      transponder(transponder)
{
    DEBUGLOG_PRINTLN(F("CAN initialized"));
}

CAN::~CAN()
{
}

void CAN::onStartupFail()
{
    DEBUGLOG_PRINTLN(F("CAN startup FAIL"));
    transponder->setError(Transponder::can_fail);
    transponder->setKeyBacklight(0); // Turn off key backlight on CAN failure
}

bool CAN::instrumentBegin()
{
    // Beide RX-Buffer vergleichen alle ID-Bits
    canBus->init_Mask(0, 0, MASK_EXACT); // RXB0
    canBus->init_Mask(1, 0, MASK_EXACT); // RXB1

    // RXB0: ID 0x270
    canBus->init_Filt(0, 0, CAN_STD_ID(CanMessageId::transponder));
    canBus->init_Filt(1, 0, CAN_STD_ID(CanMessageId::transponder)); // zweiter Filter optional identisch

    // RXB1: Lights (0x203) und Gateway Heartbeat (0x300)
    canBus->init_Filt(2, 0, CAN_STD_ID(CanMessageId::lights));
    canBus->init_Filt(3, 0, CAN_STD_ID(CanMessageId::gatewayHeartbeat));
    // Optional: weitere Filter-Slots frei lassen / duplizieren (je nach MCP2515-Lib erforderlich)
    canBus->init_Filt(4, 0, CAN_STD_ID(CanMessageId::lights));
    canBus->init_Filt(5, 0, CAN_STD_ID(CanMessageId::gatewayHeartbeat));

    canBus->setMode(MCP_NORMAL);

    return true;
}

void CAN::onGatewayHeartbeatTimeout()
{
    DEBUGLOG_PRINTLN(F("Gateway heartbeat TIMEOUT"));
    transponder->setError(Transponder::can_gateway_timeout);
    transponder->setKeyBacklight(0); // Turn off key backlight on CAN failure
}

void CAN::onGatewayHeartbeatDiscovered()
{
    DEBUGLOG_PRINTLN(F("Gateway heartbeat OK"));
    transponder->setError(Transponder::no_error);
    transponder->setKeyBacklight(20); // Restore key backlight on CAN OK
}

void CAN::handleFrame(CanMessageId id, uint8_t ext, uint8_t len, const uint8_t *data)
{
        // We currently expect standard frames only (ext == 0).
    (void)ext;

    // Gateway Heartbeat is already handled by InstrumentCAN base class
    switch (id)
    {
    case CanMessageId::transponder:
    {
        if (len >= 8)
        {
            const uint16_t code = (static_cast<uint16_t>(data[0]) << 8) | static_cast<uint16_t>(data[1]);
            const uint8_t mode = data[2];
            const uint8_t light = data[3];

            transponder->setSquawkCode(String(code / 1000 % 10) +
                                       String(code / 100 % 10) +
                                       String(code / 10 % 10) +
                                       String(code % 10));
            transponder->setMode(static_cast<Transponder::TransponderMode>(mode));
            transponder->setTransponderLight(light != 0);
        }
        break;
    }
    case CanMessageId::lights:
    {
        if (len >= 8)
        {
            const uint16_t radioDim1000 = (static_cast<uint16_t>(data[2]) << 8) | static_cast<uint16_t>(data[3]);
            // Not in this instrument:
            //const uint16_t panelDim1000 = (static_cast<uint16_t>(data[0]) << 8) | static_cast<uint16_t>(data[1]);
            // const uint16_t domeLightDim1000 = (static_cast<uint16_t>(data[4]) << 8) | static_cast<uint16_t>(data[5]);
            float ratio = constrain(static_cast<float>(radioDim1000) / 1000., 0., 1.);
            uint8_t pwm = static_cast<uint8_t>(ratio * 255.);
            transponder->setKeyBacklight(pwm);
        }
        break;
    }
    default:
        DEBUGLOG_PRINTLN(String(F("Handle CAN frame Id: ")) + String(static_cast<uint16_t>(id), HEX));
        break;
    }
}

void CAN::sendTransponderState(uint16_t code, uint8_t mode, uint8_t ident)
{
    uint8_t data[8] = { static_cast<uint8_t>(code >> 8),
                        static_cast<uint8_t>(code & 0xff),
                        mode,
                        ident,
                        0, 0, 0, 0 }; // Padding bytes

    sendMessage(CanMessageId::transponderInput, 8, data);
}