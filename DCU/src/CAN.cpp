#include "CAN.h"
#include "Configuration.h"
#include "DebugLog.h"

// MCP_CAN filter values for standard IDs are typically (id << 16) in mcp_can
#ifndef CAN_STD_ID
#define CAN_STD_ID(id) ((uint32_t)(id) << 16)
#endif

volatile bool CAN::canIrq = false;
CAN* CAN::instance = nullptr;

CAN::CAN()
{
    canBus = new MCP_CAN(kCanCSPin);

    // MCP2515 /INT is open-drain, active-low. Use pull-up.
    pinMode(kCanIntPin, INPUT_PULLUP);

    // Hook interrupt to wake our loop when frames arrive.
    // NOTE: class ISR needs a static entry point; we keep a single active instance.
    instance = this;
    canIrq = false;
    attachInterrupt(digitalPinToInterrupt(kCanIntPin), CAN::onCanInterrupt, FALLING); 

    DEBUGLOG_PRINTLN(F("CAN initialized"));
}

CAN::~CAN()
{
    detachInterrupt(digitalPinToInterrupt(kCanIntPin));
    instance = nullptr;
    delete canBus;
}

bool CAN::begin() {
    if (canBus->begin(MCP_STDEXT, CAN_500KBPS, MCP_8MHZ) != CAN_OK)
    {
        DEBUGLOG_PRINTLN(F("CAN init fail"));
        return false;
    }

    // Filters: receive Instrument Heartbeat (0x301) in RXB0.
    // Configure both masks to compare all bits.
    canBus->init_Mask(0, 0, 0x7FF); // RXB0 exact match on 11-bit
    canBus->init_Mask(1, 0, 0x7FF); // RXB1 exact match on 11-bit

    // RXB0: Instrument heartbeat
    canBus->init_Filt(0, 0, CAN_STD_ID(CanStateId::instrumentHeartbeat));
    canBus->init_Filt(1, 0, CAN_STD_ID(CanStateId::instrumentHeartbeat));

    // RXB1: (reserved for future inputs)
    // For now, set both filters to instrumentHeartbeat as well.
    canBus->init_Filt(2, 0, CAN_STD_ID(CanStateId::instrumentHeartbeat));
    canBus->init_Filt(3, 0, CAN_STD_ID(CanStateId::instrumentHeartbeat));
    canBus->init_Filt(4, 0, CAN_STD_ID(CanStateId::instrumentHeartbeat));
    canBus->init_Filt(5, 0, CAN_STD_ID(CanStateId::instrumentHeartbeat));

    canBus->setMode(MCP_NORMAL);
    isStarted = true;
    return true;
}

void CAN::onCanInterrupt()
{
    // Keep ISR tiny: no SPI, no Serial.
    canIrq = true;
}

void CAN::loop()
{
    // Fast path: no interrupt seen and line is high -> nothing to do.
    if (!isStarted || (!canIrq && digitalRead(kCanIntPin) == HIGH)) {
        return;
    }

    // Clear flag early; if more frames arrive while draining, ISR will set it again.
    noInterrupts();
    canIrq = false;
    interrupts();

    // Drain all pending frames. INT stays low while RX buffers contain unread frames.
    while (digitalRead(kCanIntPin) == LOW) {
        if (canBus->checkReceive() != CAN_MSGAVAIL) {
            // Sometimes INT can lag a tiny bit; break to avoid busy-loop.
            break;
        }

        unsigned long rxId = 0;
        byte ext = 0;
        byte len = 0;
        byte buf[8] = {0};

        canBus->readMsgBuf(&rxId, &ext, &len, buf);
        handleFrame(rxId, ext, len, buf);
    }

    // --- Gateway Heartbeat TX (DCU -> Instruments), 2 Hz ---
    const uint32_t now = millis();
    const uint32_t periodMs = 500;
    const uint32_t offsetMs = 0; // Gateway = 0

    if (lastGatewayHeartbeatSendMs == 0) {
        lastGatewayHeartbeatSendMs = now + offsetMs;
    }

    if ((int32_t)(now - lastGatewayHeartbeatSendMs) >= 0) {
        sendGatewayHeartbeat();
        lastGatewayHeartbeatSendMs += periodMs;
    }

    // --- Instrument heartbeat timeout checks ---
    checkInstrumentHeartbeats();
}

void CAN::handleFrame(uint32_t id, uint8_t ext, uint8_t len, const uint8_t* data)
{
    // DEBUGLOG_PRINTLN(String(F("CAN Message received: ID "))+String(id, HEX));

    // We currently expect standard frames only (ext == 0).
    (void)ext;

    // IDs from mcp_can are the actual 11-bit ID (e.g. 0x270), even though filters use (ID<<16).
    switch (static_cast<CanStateId>(id)) {
        case CanStateId::instrumentHeartbeat:
            updateInstrumentHeartbeat(len, data);
            break;
        default:
            break;
    }
}

void CAN::sendMessage(CanStateId id, uint8_t len, byte* data)
{
  // send data:  ID = 0x100, Standard CAN Frame, Data length = 8 bytes, 'data' = array of data bytes to send
  uint8_t sndStat = canBus->sendMsgBuf(static_cast<unsigned long>(id), 0, len, data);
  if (sndStat == CAN_OK)
  {
    DEBUGLOG_PRINTLN(String(F("Message Sent Successfully to id 0x"))+String(static_cast<unsigned long>(id), HEX));
  }
  else
  {
    DEBUGLOG_PRINTLN(String(F("Error Sending Message:"))+sndStat);
  }
}

void CAN::sendGatewayHeartbeat()
{
    // CAN ID 0x300 (gatewayHeartbeat), payload 8 bytes:
    // [0]=nodeId, [1]=fwMajor, [2]=fwMinor, [3]=flags, [4..7]=uptime/10ms (u32, big endian)
    byte data[8] = {0};
    data[0] = kNodeId;
    data[1] = kFwMajor;
    data[2] = kFwMinor;
    data[3] = 0x01; // bit0=OK

    const uint32_t uptime10 = millis() / 10;
    data[4] = (uint8_t)((uptime10 >> 24) & 0xFF);
    data[5] = (uint8_t)((uptime10 >> 16) & 0xFF);
    data[6] = (uint8_t)((uptime10 >> 8) & 0xFF);
    data[7] = (uint8_t)(uptime10 & 0xFF);

    sendMessage(CanStateId::gatewayHeartbeat, 8, data);
}

void CAN::updateInstrumentHeartbeat(uint8_t len, const uint8_t* data)
{
    if (len < 8) return;

    const uint8_t nodeId = data[0];
    if (nodeId >= kMaxInstrumentNodes) return;

    lastInstrumentHeartbeatMs[nodeId] = millis();
}

void CAN::checkInstrumentHeartbeats()
{
    const uint32_t now = millis();
    const uint32_t timeoutMs = 1500;

    for (uint8_t nodeId = 0; nodeId < kMaxInstrumentNodes; ++nodeId) {
        if (nodeId == kNodeId) continue; // skip gateway itself

        const bool alive = (lastInstrumentHeartbeatMs[nodeId] != 0) && (now - lastInstrumentHeartbeatMs[nodeId] <= timeoutMs);
        if (alive != instrumentAlive[nodeId]) {
            instrumentAlive[nodeId] = alive;
            // For now: log state changes. Later can propagate to USB status/annunciators.
            DEBUGLOG_PRINTLN(String(F("Instrument HB node ")) + nodeId + (alive ? F(" OK") : F(" TIMEOUT")));
        }
    }
}