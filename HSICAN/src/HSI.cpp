#include <HSI.h>
#include <Wire.h>
#include <EEPROM.h>
#include "DebugLog.h"
#include "Backlash.h"
#include "HSIGeometry.h"

HSI *HSI::instance = nullptr;
ISR(PCINT1_vect) // HDG Interrupt
{
    if (HSI::instance)
    {
        bool buttonState = digitalRead(kHdgEncoderPushButtonPin) == LOW;
        bool aPinState = digitalRead(kHdgEncoderAPin) == LOW;
        bool bPinState = digitalRead(kHdgEncoderBPin) == LOW;
       //DEBUGLOG_PRINTLN(" [PCINT1_vect (a|b|p): " + String(aPinState ? "T|":"F|") + String(bPinState ? "T|":"F|") + String(buttonState ? "T":"F") + "] ");

        if (HSI::instance->currentHdgEncoderButtonState != buttonState)
        {
            HSI::instance->currentHdgEncoderButtonState = buttonState;
        }

        uint8_t s = HSI::instance->hdgEncoderState & 0b0011;
		if (aPinState) s |= 0b0100;
		if (bPinState) s |= 0b1000;
        HSI::instance->hdgEncoderState = (s >> 2);
        switch (s) {
            case 0b0000: case 0b0101: case 0b1010: case 0b1111:
                break;
            case 0b0001: case 0b0111: case 0b1000: case 0b1110:
                HSI::instance->currentHdgEncoder++; break;
            case 0b0010: case 0b0100: case 0b1011: case 0b1101:
                HSI::instance->currentHdgEncoder--; break;
            case 0b0011: case 0b1100:
                HSI::instance->currentHdgEncoder += 2; break;
            default:
                HSI::instance->currentHdgEncoder -= 2; break;
        }
       //DEBUGLOG_PRINTLN(" [INT currentHdgEncoder:" + String(HSI::instance->currentHdgEncoder) + "] ");
    }
}
ISR(PCINT2_vect) // OBS Interrupt
{
    if (HSI::instance)
    {
        bool buttonState = digitalRead(kObsEncoderPushButtonPin) == LOW;
        bool aPinState = digitalRead(kObsEncoderAPin) == LOW;
        bool bPinState = digitalRead(kObsEncoderBPin) == LOW;
        //DEBUGLOG_PRINTLN(" [PCINT2_vect (a|b|p): " + String(aPinState ? "T|":"F|") + String(bPinState ? "T|":"F|") + String(buttonState ? "T":"F") + "] ");
        if (HSI::instance->currentObsEncoderButtonState != buttonState)
        {
            HSI::instance->currentObsEncoderButtonState = buttonState;
        }

        uint8_t s = HSI::instance->obsEncoderState & 0b0011;
		if (aPinState) s |= 0b0100;
		if (bPinState) s |= 0b1000;
        HSI::instance->obsEncoderState = (s >> 2);
        switch (s) {
            case 0b0000: case 0b0101: case 0b1010: case 0b1111:
                break;
            case 0b0001: case 0b0111: case 0b1000: case 0b1110:
                HSI::instance->currentObsEncoder++; break;
            case 0b0010: case 0b0100: case 0b1011: case 0b1101:
                HSI::instance->currentObsEncoder--; break;
            case 0b0011: case 0b1100:
                HSI::instance->currentObsEncoder += 2; break;
            default:
                HSI::instance->currentObsEncoder -= 2; break;
        }
        //DEBUGLOG_PRINTLN(" [INT currentObsEncoder:" + String(HSI::instance->currentObsEncoder) + "] ");
    }
}

HSI::HSI()
{
    loadConfig();

    // Setup zeroing pins & interrupts
    Wire.begin();
    // 100 kHz would make a one-byte register read cost roughly 400 us against a
    // 1.22 ms step interval at 12 rpm / 4096 steps. At 400 kHz it is about 100.
    Wire.setClock(400000);

    mcp = new MCP23017(kMCP23017Address);
    mcp->init();
    mcp->portMode(MCP23017Port::A, 0); // Port A as output: obs + hdg coils
    // Port B: GPB0..3 compass coils (output), GPB4..6 hall inputs with pull-ups
    // and IPOL inverted, so a set bit reads as "over the magnet". Writes to the
    // input bits only land in OLAT, so writePort(B, pattern & 0x0F) stays valid.
    mcp->portMode(MCP23017Port::B, kHallMask, kHallMask, kHallMask);

    mcp->writeRegister(MCP23017Register::GPIO_A, 0x00); // Reset port A
    mcp->writeRegister(MCP23017Register::GPIO_B, 0x00); // Reset port B (writes to the hall input bits just land in OLAT)

    // DEBUGLOG_PRINTLN("MCP23017 initialized");

    axes[compass] = new CheapStepper(&compassPattern, config.stepsBacklash[compass], true);
    axes[obs] = new CheapStepper(&obsPattern, config.stepsBacklash[obs], true);
    axes[hdg] = new CheapStepper(&hdgPattern, config.stepsBacklash[hdg], true);

    for (int axis = 0; axis < hsiAxisCount; axis++)
    {
        // DEBUGLOG_PRINTLN(String("Initializing ") + axisName(static_cast<HSIAxis>(axis)));
        // DEBUGLOG_PRINT(String("Total Steps: ") + kTotalSteps[axis]);
        // DEBUGLOG_PRINTLN(String(", MaxRPM: ") + static_cast<int>(kRpmLimits[axis][maxRpm]));

        axes[axis]->setTotalSteps(kTotalSteps[axis]);
        axes[axis]->setRpm(kRpmLimits[axis][maxRpm]);
    }

    for (int servoId = 0; servoId < servoCount; servoId++)
    {
        // DEBUGLOG_PRINTLN(String("Initializing ") + servoName(static_cast<ServoId>(servoId)));
        // DEBUGLOG_PRINTLN(String("Servo Pin: ") + static_cast<int>(kServoPins[servoId]));
        servos[servoId] = new Servo();
        servos[servoId]->attach(kServoPins[servoId]);
    }

    // Enable Encoder Pins & Interrupts
    pinMode(kObsEncoderAPin, INPUT_PULLUP);
    pinMode(kObsEncoderBPin, INPUT_PULLUP);
    pinMode(kObsEncoderPushButtonPin, INPUT_PULLUP);
    pinMode(kHdgEncoderAPin, INPUT_PULLUP);
    pinMode(kHdgEncoderBPin, INPUT_PULLUP);
    pinMode(kHdgEncoderPushButtonPin, INPUT_PULLUP);

    delay(2000); // Wait for the Pullup resistors to stabilize
    obsEncoderButtonState = digitalRead(kObsEncoderPushButtonPin) == LOW;
    currentObsEncoderButtonState = obsEncoderButtonState;
    obsEncoderState = 0;
    if(digitalRead(kObsEncoderAPin)) obsEncoderState |= 1;
    if(digitalRead(kObsEncoderBPin)) obsEncoderState |= 2;
    hdgEncoderState = 0;
    if(digitalRead(kHdgEncoderAPin)) hdgEncoderState |= 1;
    if(digitalRead(kHdgEncoderBPin)) hdgEncoderState |= 2;
    hdgEncoderButtonState = digitalRead(kHdgEncoderPushButtonPin) == LOW;
    currentHdgEncoderButtonState = hdgEncoderButtonState;

    readHalls(); // seed the cache before the initial state sweep
    for (int axis = 0; axis < hsiAxisCount; axis++) {
        fetchZeroedState(static_cast<HSIAxis>(axis));
    }

    PCICR = (1 << PCIE2) | (1 << PCIE1);
    PCMSK1 = kHdgEncoderInterruptMask;
    PCMSK2 = kObsEncoderInterruptMask;

    pinMode(kLightPin, OUTPUT);
    analogWrite(kLightPin, 0);

    instance = this;
    DEBUGLOG_PRINTLN(String(F("HSI initialized")));
}

HSI::~HSI()
{
    for (int servoId = 0; servoId < servoCount; servoId++)
    {
        servos[servoId]->detach();
        delete servos[servoId];
    }
    for (int axis = 0; axis < hsiAxisCount; axis++)
    {
        axes[axis]->off();
        delete axes[axis];
    }
    mcp->writeRegister(MCP23017Register::GPIO_A, 0x00); // Reset port A
    mcp->writeRegister(MCP23017Register::GPIO_B, 0x00); // Reset port B
    delete mcp;
}

void HSI::loadConfig()
{
    EEPROM.get(kHsiEepromAddress, config);
    if (config.magic == kHsiEepromMagic)
    {
        DEBUGLOG_PRINTLN(F("HSI: EEPROM config loaded"));
        return;
    }

    DEBUGLOG_PRINTLN(F("HSI: No valid EEPROM config, writing defaults"));
    config.magic = kHsiEepromMagic;
    for (int axis = 0; axis < hsiAxisCount; axis++)
    {
        config.zeroAdjustDegree[axis] = kDefaultZeroAdjustDegree[axis];
        config.stepsBacklash[axis] = kDefaultStepsBacklash[axis];
        config.moveDeadband[axis] = kDefaultMoveDeadband[axis];
        config.reversalDeadband[axis] = kDefaultReversalDeadband[axis];
    }
    for (int id = 0; id < servoCount; id++)
    {
        config.servoAdjustDegree100[id] = kDefaultServoAdjustDegree100[id];
        config.servoAdjustRatio1000[id] = kDefaultServoAdjustRatio1000[id];
    }
    config.cdiDegPerDot100 = kDefaultCdiDegPerDot100;
    config.gsDegPerDot100 = kDefaultGsDegPerDot100;
    EEPROM.put(kHsiEepromAddress, config);
}

void HSI::saveConfig()
{
    EEPROM.put(kHsiEepromAddress, config);
    DEBUGLOG_PRINTLN(F("HSI: EEPROM config saved"));
}

void HSI::wipeCalibration()
{
    config.magic = 0;
    EEPROM.put(kHsiEepromAddress, config);
    loadConfig();
}

void HSI::setStepsBacklash(HSIAxis axis, uint16_t steps)
{
    config.stepsBacklash[axis] = steps;
    axes[axis]->setBacklashSteps(steps);
    saveConfig();
}

void HSI::setServoAdjust(ServoId id, int16_t degree100, int16_t ratio1000)
{
    config.servoAdjustDegree100[id] = degree100;
    config.servoAdjustRatio1000[id] = ratio1000;
    saveConfig();
}

void HSI::setDegPerDot(bool isGs, int16_t degPerDot100)
{
    if (isGs)
    {
        config.gsDegPerDot100 = degPerDot100;
    }
    else
    {
        config.cdiDegPerDot100 = degPerDot100;
    }
    saveConfig();
}

bool HSI::calibrateZero(HSIAxis axis)
{
    // The axis was homed and then jogged onto its true mark by hand. Its
    // reported position is that jog, so it is added to the stored offset -
    // replacing would discard every earlier pass.
    const int32_t jog = jogDegreesFromPosition(axes[axis]->getPosition(),
                                               axes[axis]->getTotalSteps());
    config.zeroAdjustDegree[axis] = accumulateZeroAdjust(config.zeroAdjustDegree[axis], jog);
    saveConfig();
    axes[axis]->resetPosition();
    return true;
}

uint32_t HSI::measureBacklashSteps(HSIAxis axis)
{
    // Creep one way until the hall sensor switches, then reverse and count how
    // many steps pass before it switches back. Those are the steps the gear
    // train swallows. Call only with stepsBacklash already set to 0, otherwise
    // the compensation hides the very thing being measured.
    const uint32_t limit = axes[axis]->getTotalSteps() / 4;

    axes[axis]->setRpm(kRpmLimits[axis][minRpm]);

    readHalls();
    fetchZeroedState(axis);
    const bool startState = zeroedState[axis];

    // Phase 1: drive forward onto the sensor edge.
    axes[axis]->newMove(true, limit);
    while (axes[axis]->getStepsLeft() != 0)
    {
        axes[axis]->run();
        sendMotorData();
        readHalls();
        fetchZeroedState(axis);
        if (zeroedState[axis] != startState)
        {
            break;
        }
    }
    axes[axis]->stop();

    if (zeroedState[axis] == startState)
    {
        DEBUGLOG_PRINTLN(F("HSI: no edge found, aborting"));
        axes[axis]->setRpm(kRpmLimits[axis][maxRpm]);
        return 0;
    }

    const bool edgeState = zeroedState[axis];

    // Phase 2: reverse and count until the edge comes back. The slack is
    // crossed before the output moves, so those steps are the measurement.
    uint32_t steps = 0;
    axes[axis]->newMove(false, limit);
    while (axes[axis]->getStepsLeft() != 0)
    {
        axes[axis]->run();
        sendMotorData();
        steps++;
        readHalls();
        fetchZeroedState(axis);
        if (zeroedState[axis] != edgeState)
        {
            break;
        }
    }
    axes[axis]->stop();

    if (zeroedState[axis] == edgeState)
    {
        DEBUGLOG_PRINTLN(F("HSI: edge did not return, aborting"));
        axes[axis]->setRpm(kRpmLimits[axis][maxRpm]);
        return 0;
    }

    // Put the speed back: leaving the axis at minRpm would make every later
    // move on it crawl until the next homing pass.
    axes[axis]->setRpm(kRpmLimits[axis][maxRpm]);

    return steps;
}

void HSI::loop()
{
    if(instance == NULL)
    {
        return;
    }

    readHalls();

    uint32_t uS = micros();
    for (int axis = 0; axis < hsiAxisCount; axis++)
    {
        axes[axis]->run(uS);
    }

    if (isHoming())
    {
        runHoming();
    }

    sendMotorData();

    if (hdgEncoder != currentHdgEncoder)
    {
        hdgEncoder = currentHdgEncoder;
        //DEBUGLOG_PRINTLN(" [hdgEncoder:" + String(hdgEncoder) + "] ");
    }
    if (obsEncoder != currentObsEncoder)
    {
        obsEncoder = currentObsEncoder;
        //DEBUGLOG_PRINTLN(" [obsEncoder:" + String(obsEncoder) + "] ");
    }
    const uint32_t debounceDelay = 50000; // microseconds
    const uint32_t longPressDelay = 1500000; // microseconds

    // Debouncing the OBS encoder push button state
    if (currentObsEncoderButtonState != obsEncoderButtonState && (uS - lastObsDebounceTime) > debounceDelay)
    {
        lastObsDebounceTime = uS;
        obsEncoderButtonState = currentObsEncoderButtonState;
        //DEBUGLOG_PRINTLN(" [obsEncoderButtonState:" + String(obsEncoderButtonState) + "] ");
    }

    if (currentHdgEncoderButtonState != hdgEncoderButtonState &&
        (uS - lastHdgDebounceTime) > debounceDelay)
    {
        lastHdgDebounceTime = uS;
        hdgEncoderButtonState = currentHdgEncoderButtonState;

        if (hdgEncoderButtonState)
        {
            // Press: remember when, decide on release. Measuring duration is the
            // point - the old test compared against the last edge, so any press
            // after 1.5s of quiet counted as long.
            hdgPressStartedAt = uS;
            hdgPressActive = true;
        }
        else if (hdgPressActive)
        {
            hdgPressActive = false;
            if ((uS - hdgPressStartedAt) >= longPressDelay)
            {
                beginHoming();
            }
            else
            {
                hdgShortPressPending = true;
            }
        }
    }
}

bool HSI::takeObsDelta(int16_t *delta)
{
    noInterrupts();
    const int16_t now = currentObsEncoder;
    interrupts();

    const int16_t diff = now - lastSentObsEncoder;
    if (diff == 0)
    {
        return false;
    }
    lastSentObsEncoder = now;
    *delta = diff;
    return true;
}

bool HSI::takeHdgDelta(int16_t *delta)
{
    noInterrupts();
    const int16_t now = currentHdgEncoder;
    interrupts();

    const int16_t diff = now - lastSentHdgEncoder;
    if (diff == 0)
    {
        return false;
    }
    lastSentHdgEncoder = now;
    *delta = diff;
    return true;
}

bool HSI::takeHdgShortPress()
{
    if (!hdgShortPressPending)
    {
        return false;
    }
    hdgShortPressPending = false;
    return true;
}

void HSI::sendMotorData()
{
    uint8_t portA = hdgPattern << 4 | (obsPattern & 0x0F); // Combine the patterns for port A
    if (currentPortA != portA)
    {
        currentPortA = portA;
        mcp->writePort(MCP23017Port::A, portA);
    }
    uint8_t portB = compassPattern & 0x0F; // Combine the patterns for port B
    if (currentPortB != portB)
    {
        currentPortB = portB;
        mcp->writePort(MCP23017Port::B, portB);
    }
}

void HSI::stopAllAxes()
{
    for (int axis = 0; axis < hsiAxisCount; axis++)
    {
        axes[axis]->stop();
    }
}

void HSI::offAllAxes()
{
    for (int axis = 0; axis < hsiAxisCount; axis++)
    {
        axes[axis]->off();
    }
}

void HSI::beginHoming()
{
    DEBUGLOG_PRINTLN(F("HSI: homing"));
    isHomed = false;
    homingFailure = false;
    for (int axisIndex = 0; axisIndex < hsiAxisCount; axisIndex++)
    {
        HSIAxis axis = static_cast<HSIAxis>(axisIndex);
        axes[axis]->stop();
        axes[axis]->setRpm(kRpmLimits[axis][minRpm]);
        axes[axis]->newMove(axes[axis]->lastDirectionIsClockwise(),
                            axes[axis]->getBacklashSteps());
        homingState[axis] = takeUpBacklash;
    }
}

bool HSI::isHoming() const
{
    if (isHomed || homingFailure)
    {
        return false;
    }
    for (int axis = 0; axis < hsiAxisCount; axis++)
    {
        if (homingState[axis] != homed)
        {
            return true;
        }
    }
    return false;
}

bool HSI::homingFailed() const
{
    return homingFailure;
}

void HSI::runHoming()
{
    // Driven from loop(): a three-axis sweep takes seconds, and blocking
    // through it would let both RX buffers fill and the gateway heartbeat
    // expire.
    for (int axisIndex = 0; axisIndex < hsiAxisCount; axisIndex++)
    {
        HSIAxis axis = static_cast<HSIAxis>(axisIndex);
        HSIDriveResult result = nextHomingState(axis);
        if (result != success)
        {
            DEBUGLOG_PRINT(F("HSI: homing failed on axis "));
            DEBUGLOG_PRINTLN(axisIndex);
            homingFailure = true;
            homingState[axis] = timeout;
            return;
        }
    }
    if (checkAllHomed())
    {
        isHomed = true;
        DEBUGLOG_PRINTLN(F("HSI: homed"));
    }
}

void HSI::applyHeading(uint16_t heading100, uint16_t obs100, uint16_t hdgBug100)
{
    if (!isHomed)
    {
        return;
    }
    const AxisTargets t = axisTargets(heading100, obs100, hdgBug100);
    moveToDegree(compass, t.compassAxis100 / 100.0);
    moveToDegree(obs, t.obsAxis100 / 100.0);
    moveToDegree(hdg, t.hdgAxis100 / 100.0);
}

void HSI::applyFlags(uint8_t flags)
{
    // bit0 TO, bit1 FROM, bit2 GS valid.
    FromTo fromTo = noNav;
    if (flags & 0x01)
    {
        fromTo = to;
    }
    else if (flags & 0x02)
    {
        fromTo = from;
    }
    moveServo(fromToServo, static_cast<double>(fromTo));

    gsValid = (flags & 0x04) != 0;
    if (!gsValid)
    {
        moveServo(gs1Servo, 0.0);
        moveServo(gs2Servo, 0.0);
    }
}

void HSI::applyDeviation(int16_t cdiDots1000, int16_t gsDots1000)
{
    moveServo(cdiServo, dotsToServoDegree(cdiDots1000, config.cdiDegPerDot100));

    if (!gsValid)
    {
        return;
    }
    // Both GS servos carry one logical channel; only the linkage differs.
    const double gs = -dotsToServoDegree(gsDots1000, config.gsDegPerDot100);
    moveServo(gs1Servo, gs);
    moveServo(gs2Servo, gs);
}

void HSI::park()
{
    // What bus_volts < 6 did in the old AirManager LUA: card to north, the
    // deviation needles to zero, the flag to noNav. The two pointers keep their
    // positions - the LUA did not blank those either.
    moveToDegree(compass, 0.0);
    moveServo(cdiServo, 0.0);
    moveServo(gs1Servo, 0.0);
    moveServo(gs2Servo, 0.0);
    moveServo(fromToServo, static_cast<double>(noNav));
}

HSI::HSIDriveResult HSI::moveToDegree(HSIAxis axis, double degree)
{
    DEBUGLOG_PRINT(String("MoveToDegree: ")+axisName(axis)+String(" to ")+String(degree));
    if (homingState[axis] != homed)
    {
        DEBUGLOG_PRINTLN(String(F(" >> Not Homed")));
        return HSIDriveResult::notHomed;
    }

    int32_t targetPosition = static_cast<int32_t>(degree * static_cast<double>(axes[axis]->getTotalSteps()) / static_cast<double>(DEGREE_FULL_ROTATION));
    int32_t stepsToMove = calculateShortestPath(axis, targetPosition);

    // 0x104 arrives at 50 Hz in hundredths of a degree, so the integer target
    // jitters by +-1 step continuously. Issuing those would append a full slack
    // crossing on every flip - up to 75 steps on the heading bug - and a
    // standing pointer would buzz back and forth forever.
    if (!moveIsWorthMaking(stepsToMove,
                           axes[axis]->lastDirectionIsClockwise(),
                           config.moveDeadband[axis],
                           config.reversalDeadband[axis]))
    {
        return success;
    }

    moveSteps(axis, stepsToMove);
    return success;
}

HSI::HSIDriveResult HSI::moveServo(ServoId id, double degree)
{
    if (id >= ServoId::servoCount)
    {
        return invalidId;
    }
    const double adjusted = servoTargetDegree(degree,
                                              kServoMinimumDegree[id],
                                              kServoMaximumDegree[id],
                                              config.servoAdjustRatio1000[id],
                                              config.servoAdjustDegree100[id]);
    DEBUGLOG_PRINTLN(String(F("Move ")) + servoName(id) + String(F(" to ")) + String(degree) + String(F(" adjusted to ")) + String(adjusted));

    servos[id]->write(adjusted);
    return success;
}

void HSI::setBrightness(uint8_t pwm)
{
    analogWrite(kLightPin, pwm);
}

void HSI::moveSteps(HSIAxis axis, int16_t steps, bool synchron)
{
    axes[axis]->newMove(steps > 0, static_cast<uint32_t>(abs(steps)));
    while (synchron && axes[axis]->getStepsLeft() != 0)
    {
        axes[axis]->run();
        sendMotorData();
    }
}

void HSI::moveDegree(HSIAxis axis, int16_t degree, bool synchron)
{
    int32_t numSteps = static_cast<int32_t>(degree) * static_cast<int32_t>(axes[axis]->getTotalSteps()) / DEGREE_FULL_ROTATION;
    moveSteps(axis, numSteps, synchron);
}

int32_t HSI::calculateShortestPath(HSIAxis axis, int32_t targetPosition)
{
    // Normalize targetStep to the range [0, totalSteps)
    uint32_t normalizedTargetPosition = normalizePosition(targetPosition, axes[axis]->getTotalSteps());

    // Berechne die Differenzen in beide Richtungen
    uint32_t diffCW = (normalizedTargetPosition - axes[axis]->getPosition() + axes[axis]->getTotalSteps()) % axes[axis]->getTotalSteps();  // Clockwise
    uint32_t diffCCW = (axes[axis]->getPosition() - normalizedTargetPosition + axes[axis]->getTotalSteps()) % axes[axis]->getTotalSteps(); // Counterclockwise

    // Choose the shortest path
    if (diffCW <= diffCCW)
    {
        return static_cast<int32_t>(diffCW); // Positiv für CW-Drehung
    }
    else
    {
        return -static_cast<int32_t>(diffCCW); // Negativ für CCW-Drehung
    }
}

uint32_t HSI::normalizePosition(int32_t position, uint32_t totalSteps)
{
    return static_cast<uint32_t>((position % totalSteps + totalSteps) % totalSteps);
}

HSI::HSIDriveResult HSI::lookForZeroChangeNonBlocking(HSIAxis axis, bool targetZeroedState)
{
    fetchZeroedState(axis);
    if (zeroedState[axis] != targetZeroedState && axes[axis]->getStepsLeft() != 0)
    {
        return success;
    }
    if (zeroedState[axis] != targetZeroedState)
    {
        DEBUGLOG_PRINTLN(String(axisName(axis)) + String(F("   *** Timeout")));
        return homingTimeout;
    }
    return axisStateReached;
}

HSI::HSIDriveResult HSI::nextHomingState(HSIAxis axis)
{
    HSI::HSIDriveResult zeroState;
    switch (homingState[axis])
    {
    case takeUpBacklash:
        // Press the gear train against one flank before anything is measured.
        // Issued as a normal timed move so the motor can actually follow it;
        // the old tight-loop version outran the motor and slipped.
        if (axes[axis]->getStepsLeft() != 0)
        {
            return success;
        }
        axes[axis]->resetPosition();
        homingState[axis] = unknown;
        return success;
    case unknown:
        fetchZeroedState(axis);
        DEBUGLOG_PRINTLN(String(axisName(axis)) + String(F("- Leave zero state, current state: ")) + (zeroedState[axis] ? "true" : "false"));
        axes[axis]->stop();
        axes[axis]->resetPosition();
        axes[axis]->setRpm(kRpmLimits[axis][maxRpm]);
        moveDegree(axis, 370);
        homingState[axis] = leaveZero;
        return success;
    case leaveZero:
        zeroState = lookForZeroChangeNonBlocking(axis, false);
        if (zeroState == axisStateReached)
        {
            DEBUGLOG_PRINTLN(String(axisName(axis)) + String(F("- Search zero state, current state: ")) + (zeroedState[axis] ? "true" : "false"));
            axes[axis]->stop();
            axes[axis]->resetPosition();
            moveDegree(axis, 370);
            homingState[axis] = searchZero;
            return success;
        }
        return zeroState;

    case searchZero:
        zeroState = lookForZeroChangeNonBlocking(axis, true);
        if (zeroState == axisStateReached)
        {
            DEBUGLOG_PRINTLN(String(axisName(axis)) + String(F("- Slow search end of zero state, current state: ")) + (zeroedState[axis] ? "true" : "false"));
            axes[axis]->stop();
            axes[axis]->resetPosition();
            axes[axis]->setRpm(kRpmLimits[axis][minRpm]);
            moveDegree(axis, 60);
            homingState[axis] = searchZeroEnd;
            return success;
        }
        return zeroState;
    case searchZeroEnd:
        zeroState = lookForZeroChangeNonBlocking(axis, false);
        if (zeroState == axisStateReached)
        {
            DEBUGLOG_PRINTLN(String(axisName(axis)) + String(F("- Return to zero end position, current state: ")) + (zeroedState[axis] ? "true" : "false"));
            zeroEndPosition[axis] = axes[axis]->getPosition();
            axes[axis]->stop();
            axes[axis]->resetPosition();
            moveDegree(axis, -65);
            homingState[axis] = returnToZeroEnd;
            return success;
        }
        return zeroState;
    case returnToZeroEnd:
        zeroState = lookForZeroChangeNonBlocking(axis, true);
        if (zeroState == axisStateReached)
        {
            DEBUGLOG_PRINTLN(String(axisName(axis)) + String(F("- Slow search start of zero state, current state: ")) + (zeroedState[axis] ? "true" : "false"));
            zeroEndPosition[axis] = axes[axis]->getPosition();
            axes[axis]->stop();
            axes[axis]->resetPosition();
            moveDegree(axis, -65);
            homingState[axis] = searchZeroStart;
            return success;
        }
        return zeroState;
    case searchZeroStart:
        zeroState = lookForZeroChangeNonBlocking(axis, false);
        if (zeroState == axisStateReached)
        {
            uint32_t zeroStartPosition = axes[axis]->getPosition();
            axes[axis]->stop();
            axes[axis]->resetPosition();
            axes[axis]->setRpm(kRpmLimits[axis][maxRpm]);

            // homingWindowCentre yields half the window *width* — a relative
            // offset to move by from where the axis now stands, not an absolute
            // position. moveSteps below consumes it as exactly that.
            int32_t zeroAdjust = 0;
            if (!homingWindowCentre(zeroEndPosition[axis], zeroStartPosition,
                                    axes[axis]->getTotalSteps(), &zeroAdjust))
            {
                // The two edges came back in the wrong order or too far apart.
                // The old unsigned expression would have wrapped that into half
                // a turn and homed 180 degrees out while reporting success.
                DEBUGLOG_PRINTLN(F("HSI: implausible hall window"));
                return homingTimeout;
            }

            moveSteps(axis, zeroAdjust);
            homingState[axis] = moveToTrueZero;
            return success;
        }
        return zeroState;
    case moveToTrueZero:
        if (axes[axis]->getStepsLeft() != 0)
        {
            return success;
        }
        DEBUGLOG_PRINTLN(String(axisName(axis)) + String(F("- Move to adjusted zero position, adjustment degree: ")) + String(config.zeroAdjustDegree[axis]));
        axes[axis]->resetPosition();
        moveDegree(axis, config.zeroAdjustDegree[axis]);
        homingState[axis] = moveToAdjustedZero;
        return success;
    case moveToAdjustedZero:
        if (axes[axis]->getStepsLeft() != 0)
        {
            return success;
        }
        axes[axis]->resetPosition();
        homingState[axis] = homed;
        DEBUGLOG_PRINTLN(String(axisName(axis)) + String(F("- Homed")));
        return success;
    case homed:
        return success;
    case timeout:
        return homingTimeout;
    }
    return homingError;
}

bool HSI::checkAllHomed()
{
    for (int i = 0; i < hsiAxisCount; i++)
    {
        if (homingState[i] != homed)
        {
            return false;
        }
    }
    return true;
}

void HSI::readHalls()
{
    // One transaction serves all three axes. Called once per loop() rather than
    // once per fetchZeroedState(), which used to poll a pin per axis.
    hallBits = mcp->readPort(MCP23017Port::B) & kHallMask;
}

void HSI::fetchZeroedState(HSIAxis axis)
{
    // IPOL already inverted the active-low sensor, so a set bit is "at zero".
    bool state = (hallBits & kHallBits[axis]) != 0;
    if (state != zeroedState[axis])
    {
        zeroedState[axis] = state;
        DEBUGLOG_PRINTLN(String(F("    > Zeroed state changed on ")) + axisName(axis) + String(F(" axis: ")) + String(state ? F("true") : F("false")));
    }
}

String HSI::axisName(HSIAxis axis)
{
    switch (axis)
    {
    case obs:
        return "OBS";
    case compass:
        return "Compass";
    case hdg:
        return "HDG";
    default:
        return "Unknown";
    }
}

String HSI::servoName(ServoId id)
{
    switch (id)
    {
    case cdiServo:
        return "CDI Servo";
    case fromToServo:
        return "FromTo Servo";
    case gs1Servo:
        return "GS1 Servo";
    case gs2Servo:
        return "GS2 Servo";
    default:
        return "Unknown";
    }
}

String HSI::errorName(HSIDriveResult result)
{
    switch (result)
    {
    case success:
        return "Success";
    case homingError:
        return "Homing Error";
    case notHomed:
        return "Not Homed";
    case invalidId:
        return "Invalid ID";
    case homingTimeout:
        return "Homing Timeout";
    default:
        return "Unknown Error";
    }
}
