#include "CAN.h"
#include "Configuration.h"
#include "DebugLog.h"
#if MOTION_TESTBENCH
#include "TestBench.h"
#endif

CAN::CAN(MotionActor *motionActor)
    : BaseCAN(kCanCSPin, kCanIntPin, {static_cast<uint16_t>(kNodeId), 1, 0}),
      motionActor(motionActor)
{
    pinMode(kRedLEDPin, OUTPUT);
    pinMode(kGreenLEDPin, OUTPUT);
    digitalWrite(kRedLEDPin, HIGH);
    digitalWrite(kGreenLEDPin, LOW);

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

    // RXB0: exact matches for demand + gateway heartbeat
    canBus->init_Mask(0, 0, MASK_EXACT);
    canBus->init_Filt(0, 0, CAN_STD_ID(MotionMessageId::actorPairDemand));
    canBus->init_Filt(1, 0, CAN_STD_ID(MotionMessageId::gatewayHeartbeat));

    // RXB1: command range 0x380-0x38F (home/stop/calibration/save messages)
    const uint32_t MASK_38X = (0x07F0UL << 16);
    canBus->init_Mask(1, 0, MASK_38X);
    canBus->init_Filt(2, 0, (0x0380UL << 16));
    canBus->init_Filt(3, 0, (0x0380UL << 16));
    canBus->init_Filt(4, 0, (0x0380UL << 16));
    canBus->init_Filt(5, 0, (0x0380UL << 16));

    canBus->setMode(MCP_NORMAL);

    isStarted = true;
    DEBUGLOG_PRINTLN(F("CAN did begin"));
    return true;
}

void CAN::loop()
{
    if (!isStarted)
    {
        updateStatusLeds();
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
    const bool alive = gatewayHeartbeatSeen && (now - lastGatewayHeartbeat <= GATEWAY_TIMEOUT);
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

    updateStatusLeds();

#if MOTION_TESTBENCH
    // Must tick every pass: the fast-path return below fires on every loop
    // without CAN traffic, which would throttle the bench to the 2 Hz gateway
    // heartbeat cadence.
    if (testBench != nullptr)
    {
        testBench->loop();
    }
#endif

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

    // Apply the newest coalesced demand outside the drain loop; setDemands() blocks on
    // the Kangaroo serial round-trip.
    if (demandPending)
    {
        demandPending = false;
#if MOTION_TESTBENCH
        // During a generated test run the bench owns the motors - incoming 0x110
        // demands must not fight the strategy under test. Passthrough (strategy 9)
        // instruments this very path instead.
        const bool benchOwnsMotors =
            testBench != nullptr && testBench->isRunning() && !testBench->isPassthroughActive();
        if (!benchOwnsMotors)
        {
            const uint32_t cmdStartUs = micros();
            motionActor->setDemands(pendingDemand1, pendingDemand2);
            if (testBench != nullptr)
            {
                testBench->noteDemandApplied(micros() - cmdStartUs);
            }
        }
#else
        motionActor->setDemands(pendingDemand1, pendingDemand2);
#endif
    }
}

void CAN::resetHeartbeatClocks()
{
    // Called after a long blocking operation (homing takes seconds). Grant the gateway a
    // fresh timeout window instead of tripping on the stale timestamp, and reschedule our
    // own heartbeat so the catch-up logic doesn't fire a burst of frames.
    const uint32_t now = millis();
    lastGatewayHeartbeat = now;
    lastActorHeartbeat = now + (uint32_t)fwInfo.nodeId * 20;
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

    gatewayHeartbeatSeen = true;
    lastGatewayHeartbeat = millis();

    // Optional: you could parse version/flags/uptime here if needed.
}

void CAN::updateStatusLeds()
{
    if (motionActor->state == MotionActorState::homing)
    {
        const uint32_t now = millis();
        if ((now - lastLedBlinkToggle) >= LED_BLINK_INTERVAL)
        {
            lastLedBlinkToggle = now;
            ledBlinkStateOn = !ledBlinkStateOn;
        }

        digitalWrite(kRedLEDPin, ledBlinkStateOn ? HIGH : LOW);
        digitalWrite(kGreenLEDPin, ledBlinkStateOn ? HIGH : LOW);
        return;
    }

    const bool isReadyForDemand =
        isStarted &&
        gatewayAlive &&
        (motionActor->state == MotionActorState::active);

    digitalWrite(kGreenLEDPin, isReadyForDemand ? HIGH : LOW);
    digitalWrite(kRedLEDPin, isReadyForDemand ? LOW : HIGH);
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
            // Coalesce: only remember the newest demand here. setDemands() blocks on the
            // Kangaroo serial round-trip, which must not happen inside the RX drain loop.
            pendingDemand1 = (static_cast<uint16_t>(data[1]) << 8) | static_cast<uint16_t>(data[2]);
            pendingDemand2 = (static_cast<uint16_t>(data[3]) << 8) | static_cast<uint16_t>(data[4]);
            demandPending = true;
            DEBUGLOG_PRINTLN(String(F("Received demands: ")) + pendingDemand1 + ", " + pendingDemand2);
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
            demandPending = false; // drop any demand queued before homing
            motionActor->home();
            resetHeartbeatClocks();
        }
        break;
    case MotionMessageId::actorPairStop:
        if (len >= 1 && data[0] == static_cast<uint8_t>(kNodeId))
        {
            DEBUGLOG_PRINTLN(F("Received stop command"));
            motionActor->powerDown();
        }
        break;
    case MotionMessageId::actorCalibrationMove:
        if (len >= 4 && data[0] == static_cast<uint8_t>(kNodeId))
        {
            const uint8_t channel = data[1];
            const uint16_t targetPercent = (static_cast<uint16_t>(data[2]) << 8) | static_cast<uint16_t>(data[3]);
            DEBUGLOG_PRINTLN(String(F("Received calibration move channel=")) + channel + String(F(" target=")) + targetPercent);
            motionActor->calibrationMove(channel, targetPercent);
        }
        break;
#if MOTION_TESTBENCH
    case MotionMessageId::actorTestStart:
        if (len >= 8 && data[0] == static_cast<uint8_t>(kNodeId) && testBench != nullptr)
        {
            demandPending = false; // the bench owns the motors during a generated run
            testBench->startTest(len, data);
        }
        break;
    case MotionMessageId::actorTestAbort:
        if (len >= 1 && data[0] == static_cast<uint8_t>(kNodeId) && testBench != nullptr)
        {
            testBench->abortTest();
        }
        break;
    case MotionMessageId::actorTestDumpRequest:
        if (len >= 1 && data[0] == static_cast<uint8_t>(kNodeId) && testBench != nullptr)
        {
            testBench->requestDump();
        }
        break;
#endif
    case MotionMessageId::actorSaveLogicMin:
        if (len >= 2 && data[0] == static_cast<uint8_t>(kNodeId))
        {
            const uint8_t channel = data[1];
            DEBUGLOG_PRINTLN(String(F("Received save logical min channel=")) + channel);
            motionActor->saveLogicalMin(channel);
        }
        break;
    case MotionMessageId::actorSaveLogicMax:
        if (len >= 2 && data[0] == static_cast<uint8_t>(kNodeId))
        {
            const uint8_t channel = data[1];
            DEBUGLOG_PRINTLN(String(F("Received save logical max channel=")) + channel);
            motionActor->saveLogicalMax(channel);
        }
        break;
    default:
        DEBUGLOG_PRINTLN(String(F("Handle CAN frame Id: ")) + String(static_cast<uint16_t>(id), HEX));
        break;
    }
}