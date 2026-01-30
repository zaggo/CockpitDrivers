#include "InstrumentCAN.h"

InstrumentCAN::InstrumentCAN(uint8_t csPin, uint8_t intPin, CANFirmwareInfo fwInfo)
    : BaseCAN(csPin, intPin, fwInfo)
{
}

InstrumentCAN::~InstrumentCAN()
{
}

bool InstrumentCAN::begin()
{
    if (!BaseCAN::begin())
    {
        onStartupFail();
        return false;
    }

    if (!instrumentBegin()) {
       onStartupFail();
       return false;
    }
    
    // // Setup filters for instrument-relevant messages
    // // Filter for gateway heartbeat and other common messages
    // canBus->init_Mask(0, 0, 0x7FF); // Mask 0 for RXB0
    // canBus->init_Filt(0, 0, static_cast<unsigned long>(CanMessageId::gatewayHeartbeat));
    // canBus->init_Filt(1, 0, static_cast<unsigned long>(CanMessageId::fuelLevel));

    // canBus->init_Mask(1, 0, 0x7FF); // Mask 1 for RXB1
    // canBus->init_Filt(2, 0, static_cast<unsigned long>(CanMessageId::lights));
    isStarted = true;
    return true;
}

void InstrumentCAN::loop()
{
    if (!isStarted)
    {
        return;
    }

    // --- Heartbeat TX (Instrument -> DCU), 2 Hz mit kleinem Offset pro Node ---
    const uint32_t now = millis();
    const uint32_t offsetMs = (uint32_t)fwInfo.nodeId * 20; // vermeidet gleichzeitige HBs

    if (lastInstrumentHeartbeat == 0)
    {
        lastInstrumentHeartbeat = now + offsetMs; // erster Sendetermin
    }

    if ((int32_t)(now - lastInstrumentHeartbeat) >= 0)
    {
        sendInstrumentHeartbeat();
        lastInstrumentHeartbeat += HEARTBEAT_INTERVAL;
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

        if (rxId == static_cast<unsigned long>(CanMessageId::gatewayHeartbeat))
        {
            updateGatewayHeartbeat(len, buf);
        }
        else
        {
            handleFrame(static_cast<CanMessageId>(rxId), ext, len, buf);
        }
    }
}

void InstrumentCAN::sendInstrumentHeartbeat()
{
    // CAN ID 0x301 (instrumentHeartbeat), payload 8 bytes:
    // [0]=nodeId, [1]=fwMajor, [2]=fwMinor, [3]=flags, [4..7]=uptime/10ms (u32, big endian)
    byte data[8] = {0};
    data[0] = fwInfo.nodeId;
    data[1] = fwInfo.fwMajor;
    data[2] = fwInfo.fwMinor;
    data[3] = gatewayAlive ? 0x01 : 0x00; // bit0=OK (optional)

    const uint32_t uptime10 = millis() / 10;
    data[4] = (uint8_t)((uptime10 >> 24) & 0xFF);
    data[5] = (uint8_t)((uptime10 >> 16) & 0xFF);
    data[6] = (uint8_t)((uptime10 >> 8) & 0xFF);
    data[7] = (uint8_t)(uptime10 & 0xFF);

    sendMessage(CanMessageId::instrumentHeartbeat, 8, data);
}

void InstrumentCAN::updateGatewayHeartbeat(uint8_t len, const uint8_t *data)
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