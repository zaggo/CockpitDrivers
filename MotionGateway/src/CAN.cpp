#include "CAN.h"
#include "Configuration.h"
#include "DebugLog.h"

CAN::CAN()
    : BaseCAN(kCanCSPin, kCanIntPin, {static_cast<uint16_t>(MotionNodeId::gatewayNodeId), 1, 0})
{
    // Initialize CAN ID error tracking array
    for (uint8_t i = 0; i < kMaxCanIdErrors; ++i)
    {
        canIdErrors[i].canId = 0;
        canIdErrors[i].hasError = false;
        canIdErrors[i].errorType = CanErrorType::NONE;
    }
    canIdErrorCount = 0;

    // MCP2515 /INT is open-drain, active-low. Use pull-up.
    pinMode(kCanIntPin, INPUT_PULLUP);

    // Configure status LEDs (for system state)
    pinMode(kStatusLedRedPin, OUTPUT);
    pinMode(kStatusLedGreenPin, OUTPUT);


    // Welcome LED blink to indicate startup (can be removed later if not needed)
    // Cycle 3 times through red, green, yellow (red+green)
    for (int i = 0; i < 3; ++i)
    {
        digitalWrite(kStatusLedRedPin, HIGH);
        digitalWrite(kStatusLedGreenPin, LOW);
        delay(500);
        digitalWrite(kStatusLedRedPin, LOW);
        digitalWrite(kStatusLedGreenPin, HIGH);
        delay(500);
        digitalWrite(kStatusLedRedPin, HIGH);
        digitalWrite(kStatusLedGreenPin, HIGH);
        delay(500);
    }

    digitalWrite(kStatusLedRedPin, HIGH);
    digitalWrite(kStatusLedGreenPin, LOW);

    DEBUGLOG_PRINTLN(F("CAN initialized"));
}

CAN::~CAN()
{
    // BaseCAN destructor will handle canBus cleanup and interrupt detach
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

    uint32_t actorHeartbeat = CAN_STD_ID(MotionMessageId::actorHeartbeat);

    // RXB0: Actor heartbeat
    canBus->init_Filt(0, 0, actorHeartbeat);
    canBus->init_Filt(1, 0, actorHeartbeat);

    // RXB1: (reserved for future inputs)
    canBus->init_Filt(2, 0, actorHeartbeat);
    canBus->init_Filt(3, 0, actorHeartbeat);
    canBus->init_Filt(4, 0, actorHeartbeat);
    canBus->init_Filt(5, 0, actorHeartbeat);

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

    const uint32_t now = millis();

    // --- CAN RX: Handle incoming frames ---
    // Check if messages are available (polling mode since INT pin doesn't work reliably)
    byte receiveStatus = canBus->checkReceive();
    if (receiveStatus != CAN_NOMSG)
    {
        // Clear interrupt flag if it was set
        noInterrupts();
        canIrq = false;
        interrupts();

        // Drain all pending frames
        while (canBus->checkReceive() != CAN_NOMSG)
        {

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

    if (lastGatewayHeartbeatSendMs == 0)
    {
        lastGatewayHeartbeatSendMs = now + offsetMs;
    }

    // Uncomment to enable Gateway Heartbeat
    if ((int32_t)(now - lastGatewayHeartbeatSendMs) >= 0)
    {
        sendGatewayHeartbeat();
        lastGatewayHeartbeatSendMs += periodMs;
    }

    // --- Actor heartbeat timeout checks ---
    checkActorHeartbeats();

    // --- Calculate system state and update status LED ---
    currentSystemState = calculateSystemState();
    updateStatusLED();
}

void CAN::handleFrame(uint32_t id, uint8_t ext, uint8_t len, const uint8_t *data)
{
    // DEBUGLOG_PRINTLN(String(F("CAN Message received: ID "))+String(id, HEX));

    // We currently expect standard frames only (ext == 0).
    (void)ext;

    // Successfully received a frame - clear RX error for this CAN ID
    clearCanIdError(static_cast<uint16_t>(id), CanErrorType::RX_ERROR);

    // IDs from mcp_can are the actual 11-bit ID (e.g. 0x270), even though filters use (ID<<16).
    switch (static_cast<MotionMessageId>(id))
    {
    case MotionMessageId::actorHeartbeat:
        updateActorHeartbeat(len, data);
        break;
    default:
        break;
    }
}

void CAN::sendMessage(MotionMessageId id, uint8_t len, byte *data)
{
    const uint16_t canId = static_cast<uint16_t>(id);
    bool success = BaseCAN::sendMessage(canId, len, data);
    if (success)
    {
        // DEBUGLOG_PRINTLN(String(F("Message Sent Successfully to id 0x"))+String(static_cast<unsigned long>(id), HEX));
        //  Clear error status for this CAN ID on successful send
        clearCanIdError(canId, CanErrorType::TX_ERROR);
    }
    else
    {
        DEBUGLOG_PRINTLN(String(F("Error Sending Message to id 0x")) + String(static_cast<unsigned long>(canId), HEX));
        // Set error status for this CAN ID
        setCanIdError(canId, CanErrorType::TX_ERROR);
    }
}

void CAN::sendGatewayHeartbeat()
{
    // CAN ID 0x300 (gatewayHeartbeat), payload 8 bytes:
    // [0]=nodeId, [1]=fwMajor, [2]=fwMinor, [3]=flags, [4..7]=uptime/10ms (u32, big endian)
    byte data[8] = {0};
    data[0] = fwInfo.nodeId;
    data[1] = fwInfo.fwMajor;
    data[2] = fwInfo.fwMinor;
    data[3] = 0x01; // bit0=OK

    const uint32_t uptime10 = millis() / 10;
    data[4] = (uint8_t)((uptime10 >> 24) & 0xFF);
    data[5] = (uint8_t)((uptime10 >> 16) & 0xFF);
    data[6] = (uint8_t)((uptime10 >> 8) & 0xFF);
    data[7] = (uint8_t)(uptime10 & 0xFF);

    sendMessage(MotionMessageId::gatewayHeartbeat, 8, data);
}

void CAN::updateActorHeartbeat(uint8_t len, const uint8_t *data)
{
    // DEBUGLOG_PRINTLN(String(F("Received Actor HB")) + String(len) + F(" bytes"));
    if (len < 8)
        return;

    // CAN ID 0x301 (actorHeartbeat), payload 8 bytes:
    // [0]=nodeId, [1]=fwMajor, [2]=fwMinor, [3]=state, [4..7]=uptime/10ms (u32, big endian)
    const uint8_t nodeId = data[0];
    if (nodeId >= kMaxActorNodes)
        return;
    
    const uint8_t stateValue = data[3];
    
    // DEBUGLOG_PRINTLN(String(F("Received Actor HB from node ")) + nodeId);
    lastActorHeartbeatMs[nodeId] = millis();
    
    // Update actor state
    if (stateValue < 4) {
        actorState[nodeId] = static_cast<MotionActorState>(stateValue);
    }
}

void CAN::checkActorHeartbeats()
{
    const uint32_t now = millis();
    const uint32_t timeoutMs = 1500;
    const uint16_t actorHeartbeatId = static_cast<uint16_t>(MotionMessageId::actorHeartbeat);

    for (uint8_t nodeId = 0; nodeId < kMaxActorNodes; ++nodeId)
    {
        if (nodeId == fwInfo.nodeId)
            continue; // skip gateway itself

        const bool alive = (lastActorHeartbeatMs[nodeId] != 0) && (now - lastActorHeartbeatMs[nodeId] <= timeoutMs);
        if (alive != actorAlive[nodeId])
        {
            actorAlive[nodeId] = alive;
            // For now: log state changes. Later can propagate to USB status/annunciators.
            // DEBUGLOG_PRINTLN(String(F("Actor HB node ")) + nodeId + (alive ? F(" OK") : F(" TIMEOUT")));

            // Update error tracking: Use actorHeartbeat CAN ID with node-specific offset
            // to distinguish different nodes (ID + nodeId)
            const uint16_t nodeSpecificId = actorHeartbeatId + nodeId;
            if (alive)
            {
                clearCanIdError(nodeSpecificId, CanErrorType::HEARTBEAT_TIMEOUT);
            }
            else
            {
                setCanIdError(nodeSpecificId, CanErrorType::HEARTBEAT_TIMEOUT);
            }
        }
    }
}

void CAN::setCanIdError(uint16_t canId, CanErrorType errorType)
{
    // Check if this CAN ID already has an error entry
    for (uint8_t i = 0; i < canIdErrorCount; ++i)
    {
        if (canIdErrors[i].canId == canId)
        {
            if (!canIdErrors[i].hasError || canIdErrors[i].errorType != errorType)
            {
#if DEBUGLOG_ENABLE
                const char *errorTypeStr = (errorType == CanErrorType::TX_ERROR) ? "TX" : (errorType == CanErrorType::RX_ERROR) ? "RX"
                                                                                                                                : "HEARTBEAT";
#endif
                DEBUGLOG_PRINTLN(String(F("CAN ID 0x")) + String(canId, HEX) + F(" ") + errorTypeStr + F(" ERROR set"));
                canIdErrors[i].hasError = true;
                canIdErrors[i].errorType = errorType;
            }
            return;
        }
    }

    // Add new error entry if space available
    if (canIdErrorCount < kMaxCanIdErrors)
    {
#if DEBUGLOG_ENABLE
        const char *errorTypeStr = (errorType == CanErrorType::TX_ERROR) ? "TX" : (errorType == CanErrorType::RX_ERROR) ? "RX"
                                                                                                                        : "HEARTBEAT";
#endif
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
    for (uint8_t i = 0; i < canIdErrorCount; ++i)
    {
        if (canIdErrors[i].canId == canId && canIdErrors[i].hasError)
        {
            // Check if we should clear this error based on type filter
            if (errorType == CanErrorType::NONE || canIdErrors[i].errorType == errorType)
            {
#if DEBUGLOG_ENABLE
                const char *errorTypeStr = (canIdErrors[i].errorType == CanErrorType::TX_ERROR) ? "TX" : (canIdErrors[i].errorType == CanErrorType::RX_ERROR) ? "RX"
                                                                                                                                                              : "HEARTBEAT";
#endif
                canIdErrors[i].hasError = false;
                canIdErrors[i].errorType = CanErrorType::NONE;
                DEBUGLOG_PRINTLN(String(F("CAN ID 0x")) + String(canId, HEX) + F(" ") + errorTypeStr + F(" ERROR cleared"));
                return;
            }
        }
    }
}

SystemState CAN::calculateSystemState()
{
    // Priority 0: CAN Error - CAN not initialized or CAN ID errors exist
    if (!isStarted)
    {
        return SystemState::canError;
    }

    // Check if any CAN ID has an error
    for (uint8_t i = 0; i < canIdErrorCount; ++i)
    {
        if (canIdErrors[i].hasError)
        {
            return SystemState::canError;
        }
    }

    // Count actors by state (only count actors that are alive)
    uint8_t activeCount = 0;
    uint8_t stoppedCount = 0;
    uint8_t homingCount = 0;
    uint8_t homingFailedCount = 0;
    uint8_t aliveCount = 0;

    for (uint8_t nodeId = 0; nodeId < kMaxActorNodes; ++nodeId)
    {
        if (nodeId == fwInfo.nodeId)
            continue; // skip gateway itself

        if (actorAlive[nodeId])
        {
            aliveCount++;
            switch (actorState[nodeId])
            {
            case MotionActorState::active:
                activeCount++;
                break;
            case MotionActorState::stopped:
                stoppedCount++;
                break;
            case MotionActorState::homing:
                homingCount++;
                break;
            case MotionActorState::homingFailed:
                homingFailedCount++;
                break;
            }
        }
    }

    // Priority based logic (lower number = higher priority)
    // canError = 0, motionError = 1, homing = 2, stopping = 3, stopped = 4, active = 5

    // Priority 1: Motion Error - at least one actor failed
    if (homingFailedCount > 0)
    {
        return SystemState::motionError;
    }

    // Priority 2: Homing - at least one actor homing
    if (homingCount > 0)
    {
        return SystemState::homing;
    }

    // Priority 3: Stopping - at least one stopped, but not all
    if (stoppedCount > 0 && stoppedCount < aliveCount)
    {
        return SystemState::stopping;
    }

    // Priority 4: Stopped - all actors stopped
    if (aliveCount > 0 && stoppedCount == aliveCount)
    {
        return SystemState::stopped;
    }

    // Priority 5: Active - all actors active
    if (aliveCount > 0 && activeCount == aliveCount)
    {
        return SystemState::active;
    }

    // Default: stopped (e.g., no actors alive yet)
    return SystemState::stopped;
}

void CAN::updateStatusLED()
{
    // LED behavior based on system state:
    // - canError -> red blink
    // - motionError -> red
    // - stopped -> yellow (red + green)
    // - stopping -> yellow blink
    // - homing -> green blink
    // - active -> green

    const uint32_t now = millis();
    const uint32_t blinkPeriodMs = 500; // 500ms on, 500ms off

    bool redOn = false;
    bool greenOn = false;

    switch (currentSystemState)
    {
    case SystemState::canError:
        // Red blink
        if (now - lastStatusLedToggleMs >= blinkPeriodMs)
        {
            statusLedBlinkState = !statusLedBlinkState;
            lastStatusLedToggleMs = now;
        }
        redOn = statusLedBlinkState;
        greenOn = false;
        break;

    case SystemState::motionError:
        // Red
        redOn = true;
        greenOn = false;
        break;

    case SystemState::stopped:
        // Yellow (red + green)
        redOn = true;
        greenOn = true;
        break;

    case SystemState::stopping:
        // Yellow blink
        if (now - lastStatusLedToggleMs >= blinkPeriodMs)
        {
            statusLedBlinkState = !statusLedBlinkState;
            lastStatusLedToggleMs = now;
        }
        redOn = statusLedBlinkState;
        greenOn = statusLedBlinkState;
        break;

    case SystemState::homing:
        // Green blink
        if (now - lastStatusLedToggleMs >= blinkPeriodMs)
        {
            statusLedBlinkState = !statusLedBlinkState;
            lastStatusLedToggleMs = now;
        }
        redOn = false;
        greenOn = statusLedBlinkState;
        break;

    case SystemState::active:
        // Green
        redOn = false;
        greenOn = true;
        break;
    }

    digitalWrite(kStatusLedRedPin, redOn ? HIGH : LOW);
    digitalWrite(kStatusLedGreenPin, greenOn ? HIGH : LOW);
}

bool CAN::isSystemActive() const
{
    return currentSystemState == SystemState::active;
}