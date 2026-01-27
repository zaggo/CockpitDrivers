#include "CAN.h"
#include "Configuration.h"
#include "DebugLog.h"

volatile bool CAN::canIrq = false;
CAN* CAN::instance = nullptr;

CAN::CAN()
{
    canBus = new MCP_CAN(kCanCSPin);

    // Initialize CAN ID error tracking array
    for (uint8_t i = 0; i < kMaxCanIdErrors; ++i) {
        canIdErrors[i].canId = 0;
        canIdErrors[i].hasError = false;
        canIdErrors[i].errorType = CanErrorType::NONE;
    }
    canIdErrorCount = 0;

    // MCP2515 /INT is open-drain, active-low. Use pull-up.
    pinMode(kCanIntPin, INPUT_PULLUP);

    // Configure CAN alarm LED (on = error/not initialized)
    pinMode(kCANAlarmPin, OUTPUT);
    for (int i=0; i<3; ++i) {
        digitalWrite(kCANAlarmPin, HIGH);
        delay(100);
        digitalWrite(kCANAlarmPin, LOW);
        delay(100);
    }
    updateAlarmLED(); // LED on until CAN is successfully initialized

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
        updateAlarmLED();
        return false;
    }

    // Filters: receive Instrument Heartbeat (0x301) in RXB0.
    canBus->init_Mask(0, 0, MASK_EXACT); // RXB0 exact match
    canBus->init_Mask(1, 0, MASK_EXACT); // RXB1 exact match

    uint32_t filterValue = CAN_STD_ID(CanMessageId::instrumentHeartbeat);

    // RXB0: Instrument heartbeat
    canBus->init_Filt(0, 0, filterValue);
    canBus->init_Filt(1, 0, filterValue);

    // RXB1: (reserved for future inputs)
    canBus->init_Filt(2, 0, filterValue);
    canBus->init_Filt(3, 0, filterValue);
    canBus->init_Filt(4, 0, filterValue);
    canBus->init_Filt(5, 0, filterValue);

    canBus->setMode(MCP_NORMAL);
    isStarted = true;
    updateAlarmLED(); // Update LED state (should turn off now)
    DEBUGLOG_PRINTLN(F("CAN did begin"));
    return true;
}

void CAN::onCanInterrupt()
{
    if (instance == nullptr || instance->isStarted == false) {
        return;
    }

    // Keep ISR tiny: no SPI, no Serial.
    canIrq = true;
}

void CAN::loop()
{
    if (!isStarted) {
        return;
    }

    const uint32_t now = millis();

    // --- CAN RX: Handle incoming frames ---
    // Check if messages are available (polling mode since INT pin doesn't work reliably)
    byte receiveStatus = canBus->checkReceive();
    if (receiveStatus != CAN_NOMSG) {
        // Clear interrupt flag if it was set
        noInterrupts();
        canIrq = false;
        interrupts();

        // Drain all pending frames
        while (canBus->checkReceive() != CAN_NOMSG) {

            unsigned long rxId = 0;
            byte ext = 0;
            byte len = 0;
            byte buf[8] = {0};

            canBus->readMsgBuf(&rxId, &ext, &len, buf);
            handleFrame(rxId, ext, len, buf);
        }
    }

    // --- Gateway Heartbeat TX (DCU -> Instruments), 2 Hz ---
    const uint32_t periodMs = 500;
    const uint32_t offsetMs = 0; // Gateway = 0

    if (lastGatewayHeartbeatSendMs == 0) {
        lastGatewayHeartbeatSendMs = now + offsetMs;
    }

    // Uncomment to enable Gateway Heartbeat
    if ((int32_t)(now - lastGatewayHeartbeatSendMs) >= 0) {
        sendGatewayHeartbeat();
        lastGatewayHeartbeatSendMs += periodMs;
    }

    // --- Instrument heartbeat timeout checks ---
    checkInstrumentHeartbeats();

    // --- Update CAN alarm LED based on error status ---
    updateAlarmLED();
}

void CAN::handleFrame(uint32_t id, uint8_t ext, uint8_t len, const uint8_t* data)
{
    //DEBUGLOG_PRINTLN(String(F("CAN Message received: ID "))+String(id, HEX));

    // We currently expect standard frames only (ext == 0).
    (void)ext;

    // Successfully received a frame - clear RX error for this CAN ID
    clearCanIdError(static_cast<uint16_t>(id), CanErrorType::RX_ERROR);

    // IDs from mcp_can are the actual 11-bit ID (e.g. 0x270), even though filters use (ID<<16).
    switch (static_cast<CanMessageId>(id)) {
        case CanMessageId::instrumentHeartbeat:
            updateInstrumentHeartbeat(len, data);
            break;
        default:
            break;
    }
}

void CAN::sendMessage(CanMessageId id, uint8_t len, byte* data)
{
  // send data:  ID = 0x100, Standard CAN Frame, Data length = 8 bytes, 'data' = array of data bytes to send
  const uint16_t canId = static_cast<uint16_t>(id);
  uint8_t sndStat = canBus->sendMsgBuf(static_cast<unsigned long>(id), 0, len, data);
  if (sndStat == CAN_OK)
  {
    //DEBUGLOG_PRINTLN(String(F("Message Sent Successfully to id 0x"))+String(static_cast<unsigned long>(id), HEX));
    // Clear error status for this CAN ID on successful send
    clearCanIdError(canId, CanErrorType::TX_ERROR);
  }
  else
  {
    DEBUGLOG_PRINTLN(String(F("Error Sending Message:"))+sndStat);
    // Set error status for this CAN ID
    setCanIdError(canId, CanErrorType::TX_ERROR);
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

    sendMessage(CanMessageId::gatewayHeartbeat, 8, data);
}

void CAN::updateInstrumentHeartbeat(uint8_t len, const uint8_t* data)
{
    // DEBUGLOG_PRINTLN(String(F("Received Instrument HB")) + String(len) + F(" bytes"));
    if (len < 8) return;

    const uint8_t nodeId = data[0];
    if (nodeId >= kMaxInstrumentNodes) return;
    // DEBUGLOG_PRINTLN(String(F("Received Instrument HB from node ")) + nodeId);
    lastInstrumentHeartbeatMs[nodeId] = millis();
}

void CAN::checkInstrumentHeartbeats()
{
    const uint32_t now = millis();
    const uint32_t timeoutMs = 1500;
    const uint16_t instrumentHeartbeatId = static_cast<uint16_t>(CanMessageId::instrumentHeartbeat);

    for (uint8_t nodeId = 0; nodeId < kMaxInstrumentNodes; ++nodeId) {
        if (nodeId == kNodeId) continue; // skip gateway itself

        const bool alive = (lastInstrumentHeartbeatMs[nodeId] != 0) && (now - lastInstrumentHeartbeatMs[nodeId] <= timeoutMs);
        if (alive != instrumentAlive[nodeId]) {
            instrumentAlive[nodeId] = alive;
            // For now: log state changes. Later can propagate to USB status/annunciators.
            // DEBUGLOG_PRINTLN(String(F("Instrument HB node ")) + nodeId + (alive ? F(" OK") : F(" TIMEOUT")));
            
            // Update error tracking: Use instrumentHeartbeat CAN ID with node-specific offset
            // to distinguish different nodes (ID + nodeId)
            const uint16_t nodeSpecificId = instrumentHeartbeatId + nodeId;
            if (alive) {
                clearCanIdError(nodeSpecificId, CanErrorType::HEARTBEAT_TIMEOUT);
            } else {
                setCanIdError(nodeSpecificId, CanErrorType::HEARTBEAT_TIMEOUT);
            }
        }
    }
}

void CAN::setCanIdError(uint16_t canId, CanErrorType errorType)
{
    // Check if this CAN ID already has an error entry
    for (uint8_t i = 0; i < canIdErrorCount; ++i) {
        if (canIdErrors[i].canId == canId) {
            if (!canIdErrors[i].hasError || canIdErrors[i].errorType != errorType) {
                const char* errorTypeStr = (errorType == CanErrorType::TX_ERROR) ? "TX" : 
                                          (errorType == CanErrorType::RX_ERROR) ? "RX" : "HEARTBEAT";
                DEBUGLOG_PRINTLN(String(F("CAN ID 0x")) + String(canId, HEX) + F(" ") + errorTypeStr + F(" ERROR set"));
                canIdErrors[i].hasError = true;
                canIdErrors[i].errorType = errorType;
            }
            return;
        }
    }
    
    // Add new error entry if space available
    if (canIdErrorCount < kMaxCanIdErrors) {
        const char* errorTypeStr = (errorType == CanErrorType::TX_ERROR) ? "TX" : 
                                  (errorType == CanErrorType::RX_ERROR) ? "RX" : "HEARTBEAT";
        canIdErrors[canIdErrorCount].canId = canId;
        canIdErrors[canIdErrorCount].hasError = true;
        canIdErrors[canIdErrorCount].errorType = errorType;
        canIdErrorCount++;
        DEBUGLOG_PRINTLN(String(F("CAN ID 0x")) + String(canId, HEX) + F(" ") + errorTypeStr + F(" ERROR set"));
    }
}

void CAN::clearCanIdError(uint16_t canId, CanErrorType errorType)
{
    // Find and clear error for this CAN ID
    // If errorType is NONE, clear any error type; otherwise only clear matching error type
    for (uint8_t i = 0; i < canIdErrorCount; ++i) {
        if (canIdErrors[i].canId == canId && canIdErrors[i].hasError) {
            // Check if we should clear this error based on type filter
            if (errorType == CanErrorType::NONE || canIdErrors[i].errorType == errorType) {
                const char* errorTypeStr = (canIdErrors[i].errorType == CanErrorType::TX_ERROR) ? "TX" : 
                                          (canIdErrors[i].errorType == CanErrorType::RX_ERROR) ? "RX" : "HEARTBEAT";
                canIdErrors[i].hasError = false;
                canIdErrors[i].errorType = CanErrorType::NONE;
                DEBUGLOG_PRINTLN(String(F("CAN ID 0x")) + String(canId, HEX) + F(" ") + errorTypeStr + F(" ERROR cleared"));
                return;
            }
        }
    }
}

void CAN::updateAlarmLED()
{
    // LED should be on if:
    // 1. CAN is not started/initialized, OR
    // 2. Any CAN ID has an error status
    
    bool ledOn = false;
    
    if (!isStarted) {
        // CAN not initialized - LED on
        ledOn = true;
    } else {
        // Check if any CAN ID has an error
        for (uint8_t i = 0; i < canIdErrorCount; ++i) {
            if (canIdErrors[i].hasError) {
                ledOn = true;
                break;
            }
        }
    }
    
    digitalWrite(kCANAlarmPin, ledOn ? HIGH : LOW);
}