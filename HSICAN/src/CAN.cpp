#include "CAN.h"
#include "Configuration.h"
#include "DebugLog.h"

CAN::CAN(HSI *hsi)
    : InstrumentCAN(kCanCSPin, kCanIntPin, CANFirmwareInfo{static_cast<uint16_t>(kNodeId), 1, 0}),
      hsi(hsi)
{
    DEBUGLOG_PRINTLN(F("CAN initialized"));
}

void CAN::onStartupFail()
{
    DEBUGLOG_PRINTLN(F("CAN startup FAIL"));
    hsi->setBrightness(0);
}

bool CAN::instrumentBegin()
{
    // Both RX buffers compare every id bit.
    canBus->init_Mask(0, 0, MASK_EXACT); // RXB0
    canBus->init_Mask(1, 0, MASK_EXACT); // RXB1

    // RXB0 takes the two high-rate frames so they never queue behind a slow one.
    canBus->init_Filt(0, 0, CAN_STD_ID(CanMessageId::hsiHeading));
    canBus->init_Filt(1, 0, CAN_STD_ID(CanMessageId::hsiNav));

    // RXB1: lights and the gateway heartbeat, duplicated across the spare slots.
    canBus->init_Filt(2, 0, CAN_STD_ID(CanMessageId::lights));
    canBus->init_Filt(3, 0, CAN_STD_ID(CanMessageId::gatewayHeartbeat));
    canBus->init_Filt(4, 0, CAN_STD_ID(CanMessageId::lights));
    canBus->init_Filt(5, 0, CAN_STD_ID(CanMessageId::gatewayHeartbeat));

    canBus->setMode(MCP_NORMAL);

    hsi->setBrightness(0);

    return true;
}

void CAN::handleFrame(CanMessageId id, uint8_t ext, uint8_t len, const uint8_t *data)
{
    // No String here: heap churn plus extra stack in the deepest call path was
    // part of the stack/heap collision that froze the CAN link on other nodes.
    (void)ext;

    switch (id)
    {
    case CanMessageId::hsiHeading:
    {
        if (len >= 7)
        {
            const uint16_t heading100 = (static_cast<uint16_t>(data[0]) << 8) | data[1];
            const uint16_t obs100 = (static_cast<uint16_t>(data[2]) << 8) | data[3];
            const uint16_t bug100 = (static_cast<uint16_t>(data[4]) << 8) | data[5];

            hsi->applyFlags(data[6]);

            // Before homing the axis positions mean nothing, so a setpoint would
            // send the pointers somewhere arbitrary. applyHeading guards this too.
            if (hsi->isHomed)
            {
                hsi->applyHeading(heading100, obs100, bug100);
            }
        }
        break;
    }

    case CanMessageId::hsiNav:
    {
        if (len >= 4)
        {
            const int16_t cdi1000 = static_cast<int16_t>((static_cast<uint16_t>(data[0]) << 8) | data[1]);
            const int16_t gs1000 = static_cast<int16_t>((static_cast<uint16_t>(data[2]) << 8) | data[3]);
            hsi->applyDeviation(cdi1000, gs1000);
        }
        break;
    }

    case CanMessageId::lights:
    {
        if (len >= 8)
        {
            const uint16_t panelDim1000 = (static_cast<uint16_t>(data[0]) << 8) | data[1];
            float ratio = constrain(static_cast<float>(panelDim1000) / 1000., 0., 1.);
            panelPwm = static_cast<uint8_t>(ratio * 255.);
            panelPwmKnown = true;
            if (!hsi->homingFailed())
            {
                hsi->setBrightness(panelPwm);
            }
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
    hsi->setBrightness(0);
    hsi->park();
}

void CAN::onGatewayHeartbeatDiscovered()
{
    DEBUGLOG_PRINTLN(F("Gateway heartbeat OK"));

    // First contact with no lights frame yet: come up lit so the instrument
    // visibly joins the bus. After that, honour whatever we were last told,
    // including zero.
    hsi->setBrightness(panelPwmKnown ? panelPwm : 255);

    // First contact with the gateway is what starts the needles: homing before
    // that would drive the instrument with no sim running.
    if (!hsi->isHomed && !hsi->isHoming())
    {
        hsi->beginHoming();
    }
}

void CAN::loop()
{
    InstrumentCAN::loop();

    int16_t delta = 0;
    if (hsi->takeObsDelta(&delta))
    {
        sendHsiInput(0, 0, delta, 0);
    }
    if (hsi->takeHdgDelta(&delta))
    {
        sendHsiInput(1, 0, delta, 0);
    }
    if (hsi->takeHdgShortPress())
    {
        sendHsiInput(1, 1, 0, 1);
    }

    if (!hsi->homingFailed())
    {
        homingFailedAtMs = 0;
        return;
    }

    const uint32_t now = millis();
    if (homingFailedAtMs == 0)
    {
        homingFailedAtMs = now;
    }

    if (now - lastBlinkMs >= kHomingBlinkMs)
    {
        lastBlinkMs = now;
        blinkOn = !blinkOn;
        hsi->setBrightness(blinkOn ? 255 : 0);
    }

    // Retry rather than sitting dead. onGatewayHeartbeatDiscovered only fires on
    // the false->true edge, so with the gateway up there would otherwise be no
    // second attempt short of a power cycle.
    if (now - homingFailedAtMs >= kHomingRetryMs && gatewayAlive)
    {
        homingFailedAtMs = 0;
        hsi->beginHoming();
    }
}

void CAN::sendHsiInput(uint8_t control, uint8_t action, int16_t delta, uint8_t pushState)
{
    byte data[8] = {0};
    data[0] = control;
    data[1] = action;
    data[2] = static_cast<uint8_t>(static_cast<uint16_t>(delta) >> 8);
    data[3] = static_cast<uint8_t>(static_cast<uint16_t>(delta) & 0xFF);
    data[4] = pushState;

    sendMessage(static_cast<uint16_t>(CanMessageId::hsiInput), 8, data);
}
