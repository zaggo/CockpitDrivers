#include "CAN.h"
#include "Configuration.h"
#include "DebugLog.h"

CAN::CAN(MotionActor *motionActor)
    : BaseCAN(kCanCSPin, kCanIntPin, {static_cast<uint16_t>(kNodeId), 1, 0}),
      motionActor(motionActor)
{
    // MCP2515 /INT is open-drain, active-low. Use pull-up.
    pinMode(kCanIntPin, INPUT_PULLUP);

    DEBUGLOG_PRINTLN(F("CAN initialized"));
}

CAN::~CAN()
{
}

bool CAN::begin()
{
    bool didBegin = BaseCAN::begin();
    if (!didBegin)
    {
        DEBUGLOG_PRINTLN(F("CAN init fail"));
        return false;
    }

    // Filters: receive Actor Heartbeat (0x301) in RXB0.
    canBus->init_Mask(0, 0, MASK_EXACT); // RXB0 exact match
    canBus->init_Mask(1, 0, MASK_EXACT); // RXB1 exact match

    // Beide RX-Buffer vergleichen alle ID-Bits
    canBus->init_Mask(0, 0, MASK_EXACT); // RXB0
    canBus->init_Mask(1, 0, MASK_EXACT); // RXB1

    canBus->init_Filt(0, 0, CAN_STD_ID(MotionMessageId::actorPairDemand));
    canBus->init_Filt(1, 0, CAN_STD_ID(MotionMessageId::actorPairDemand)); // zweiter Filter optional identisch

    canBus->init_Filt(2, 0, CAN_STD_ID(MotionMessageId::actorPairHome));
    canBus->init_Filt(3, 0, CAN_STD_ID(MotionMessageId::actorPairStop));
    canBus->init_Filt(4, 0, CAN_STD_ID(MotionMessageId::gatewayHeartbeat));
    canBus->init_Filt(5, 0, CAN_STD_ID(MotionMessageId::gatewayHeartbeat));

    canBus->setMode(MCP_NORMAL);

    isStarted = true;
    DEBUGLOG_PRINTLN(F("CAN did begin"));
    return true;
}

void CAN::loop()
{
    if (!isStarted)
    {
        return;
    }

    // --- Heartbeat TX (Instrument -> DCU), 2 Hz mit kleinem Offset pro Node ---
    const uint32_t now = millis();
    const uint32_t offsetMs = (uint32_t)fwInfo.nodeId * 20; // vermeidet gleichzeitige HBs

    if (lastActorHeartbeat == 0)
    {
        lastActorHeartbeat = now + offsetMs; // erster Sendetermin
    }

    if ((int32_t)(now - lastActorHeartbeat) >= 0)
    {
        sendActorHeartbeat();
        lastActorHeartbeat += HEARTBEAT_INTERVAL;
    }

    // --- Gateway Heartbeat Timeout (DCU -> Instrument) ---
    const bool alive = /*(lastGatewayHeartbeat != 0) &&*/ (now - lastGatewayHeartbeat <= GATEWAY_TIMEOUT);
    if (alive != gatewayAlive)
    {
        gatewayAlive = alive;
        if (!gatewayAlive)
        {
            onGatewayHeartbeatTimeout();
        }
        else
        {
            onGatewayHeartbeatDiscovered();
        }
    }

    // Fast path: no interrupt seen and line is high -> nothing to do.
    if (!canIrq && digitalRead(intPin) == HIGH)
    {
        return;
    }

    // Clear flag early; if more frames arrive while draining, ISR will set it again.
    noInterrupts();
    canIrq = false;
    interrupts();

    // Drain all pending frames. INT stays low while RX buffers contain unread frames.
    while (digitalRead(intPin) == LOW)
    {
        if (canBus->checkReceive() != CAN_MSGAVAIL)
        {
            // Sometimes INT can lag a tiny bit; break to avoid busy-loop.
            break;
        }

        unsigned long rxId = 0;
        byte ext = 0;
        byte len = 0;
        byte buf[8] = {0};

        canBus->readMsgBuf(&rxId, &ext, &len, buf);

        handleFrame(static_cast<MotionMessageId>(rxId), ext, len, buf);
    }
}

void CAN::sendActorHeartbeat()
{
    // CAN ID 0x301 (actorHeartbeat), payload 8 bytes:
    // [0]=nodeId, [1]=fwMajor, [2]=fwMinor, [3]=state, [4..7]=uptime/10ms (u32, big endian)
    byte data[8] = {0};
    data[0] = static_cast<uint8_t>(fwInfo.nodeId);
    data[1] = fwInfo.fwMajor;
    data[2] = fwInfo.fwMinor;
    data[3] = static_cast<uint8_t>(motionActor->state);

    const uint32_t uptime10 = millis() / 10;
    data[4] = (uint8_t)((uptime10 >> 24) & 0xFF);
    data[5] = (uint8_t)((uptime10 >> 16) & 0xFF);
    data[6] = (uint8_t)((uptime10 >> 8) & 0xFF);
    data[7] = (uint8_t)(uptime10 & 0xFF);

    sendMessage(static_cast<uint16_t>(MotionMessageId::actorHeartbeat), 8, data);
}

void CAN::updateGatewayHeartbeat(uint8_t len, const uint8_t *data)
{
    if (len < 8)
        return;

    // Validate nodeId for gateway (expected 0). If you ever change it, adjust here.
    const uint8_t nodeId = data[0];
    if (nodeId != 0)
        return;

    lastGatewayHeartbeat = millis();

    // Optional: you could parse version/flags/uptime here if needed.
}

void CAN::onGatewayHeartbeatTimeout()
{
    DEBUGLOG_PRINTLN(F("Gateway heartbeat TIMEOUT"));
}

void CAN::onGatewayHeartbeatDiscovered()
{
    DEBUGLOG_PRINTLN(F("Gateway heartbeat OK"));
}

void CAN::handleFrame(MotionMessageId id, uint8_t ext, uint8_t len, const uint8_t *data)
{
    // We currently expect standard frames only (ext == 0).
    (void)ext;

    // Gateway Heartbeat is already handled by InstrumentCAN base class
    switch (id)
    {
    case MotionMessageId::actorPairDemand:
    {
        if (len >= 8 && data[0] == static_cast<uint8_t>(kNodeId))
        {
            const uint16_t demand1 = (static_cast<uint16_t>(data[1]) << 8) | static_cast<uint16_t>(data[2]);
            const uint16_t demand2 = (static_cast<uint16_t>(data[3]) << 8) | static_cast<uint16_t>(data[4]);
            DEBUGLOG_PRINTLN(String(F("Received demands: ")) + demand1 + ", " + demand2);
            motionActor->setDemands(demand1, demand2);
        }
        break;
    }
    case MotionMessageId::gatewayHeartbeat:
        updateGatewayHeartbeat(len, data);
        break;
    case MotionMessageId::actorPairHome:
        if (len >= 1 && data[0] == static_cast<uint8_t>(kNodeId))
        {
            DEBUGLOG_PRINTLN(F("Received home command"));
            motionActor->home();
        }
        break;
    case MotionMessageId::actorPairStop:
        if (len >= 1 && data[0] == static_cast<uint8_t>(kNodeId))
        {
            DEBUGLOG_PRINTLN(F("Received stop command"));
            motionActor->powerDown();
        }
        break;
    default:
        DEBUGLOG_PRINTLN(String(F("Handle CAN frame Id: ")) + String(static_cast<uint16_t>(id), HEX));
        break;
    }
}