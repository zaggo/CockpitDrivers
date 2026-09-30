#include <HSI.h>
#include <Wire.h>
#include "DebugLog.h"

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
    // Setup zeroing pins & interrupts
    Wire.begin();

    mcp = new MCP23017(kMCP23017Address);
    mcp->init();
    mcp->portMode(MCP23017Port::A, 0); // Port A as output
    mcp->portMode(MCP23017Port::B, 0); // Port B as output

    mcp->writeRegister(MCP23017Register::GPIO_A, 0x00); // Reset port A
    mcp->writeRegister(MCP23017Register::GPIO_B, 0x00); // Reset port B

    // DEBUGLOG_PRINTLN("MCP23017 initialized");

    axes[compass] = new CheapStepper(&compassPattern, 15, true);
    axes[obs] = new CheapStepper(&obsPattern, 28, true);
    axes[hdg] = new CheapStepper(&hdgPattern, 75, true);

    for (int axis = 0; axis < hsiAxisCount; axis++)
    {
        // DEBUGLOG_PRINTLN(String("Initializing ") + axisName(static_cast<HSIAxis>(axis)));
        // DEBUGLOG_PRINT(String("Total Steps: ") + kTotalSteps[axis]);
        // DEBUGLOG_PRINTLN(String(", MaxRPM: ") + static_cast<int>(kRpmLimits[axis][maxRpm]));

        axes[axis]->setTotalSteps(kTotalSteps[axis]);
        axes[axis]->setRpm(kRpmLimits[axis][maxRpm]);
        pinMode(kHallPins[axis], INPUT_PULLUP);
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
    for (int axis = 0; axis < hsiAxisCount; axis++) {
        zeroedState[axis] = digitalRead(kHallPins[axis]) == LOW;
    }

    PCICR = (1 << PCIE2) | (1 << PCIE1);
    PCMSK1 = kHdgEncoderInterruptMask;
    PCMSK2 = kObsEncoderInterruptMask;

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

void HSI::loop()
{
    if(instance == NULL)
    {
        return;
    }

    uint32_t uS = micros();
    for (int axis = 0; axis < hsiAxisCount; axis++)
    {
        axes[axis]->run(uS);
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

    // Debouncing the HDG encoder push button state
    if (currentHdgEncoderButtonState != hdgEncoderButtonState && (uS - lastHdgDebounceTime) > debounceDelay)
    {
        hdgEncoderButtonState = currentHdgEncoderButtonState;
        if ((uS - lastHdgDebounceTime) > longPressDelay && hdgEncoderButtonState) {
            homeAllAxis();
        }
        lastHdgDebounceTime = uS;
       //DEBUGLOG_PRINTLN(" [hdgEncoderButtonState:" + String(hdgEncoderButtonState) + "] ");
    }
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

HSI::HSIDriveResult HSI::homeAllAxis()
{
    isHomed = false;
    stopAllAxes();
    for (int axisIndex = 0; axisIndex < hsiAxisCount; axisIndex++)
    {
        HSIAxis axis = static_cast<HSIAxis>(axisIndex);
        resetPositionAndBacklash(axis); // sync
        homingState[axis] = unknown;
        nextHomingState(axis);
    }

    while (!checkAllHomed())
    {
        for (int axisIndex = 0; axisIndex < hsiAxisCount; axisIndex++)
        {
            HSIAxis axis = static_cast<HSIAxis>(axisIndex);
            HSIDriveResult result = nextHomingState(axis);
            if (result != success)
            {
                DEBUGLOG_PRINTLN(String(F("*** Error homing '")) + errorName(result) + String(F("' on axis ")) + axisName(axis));
                return result;
            }
            axes[axisIndex]->run();
        }
        sendMotorData();
    }

    // homeAxis(compass);
    // homeAxis(hdg);
    isHomed = true;
    return success;
}

HSI::HSIDriveResult HSI::moveToDegree(double obsDegree, double compassDegree, double hdgDegree, double cdiOffset, FromTo fromTo, double gsOffset)
{
    const double motorDegree[hsiAxisCount] = {
        fmod(compassDegree- obsDegree + 360.0, 360.0),
        compassDegree,
        fmod(compassDegree - hdgDegree + 360.0, 360.0)
    };

    for (int axis = 0; axis < hsiAxisCount; axis++)
    {
        moveToDegree(static_cast<HSIAxis>(axis), motorDegree[axis]);
    }

    const double servoDegree[servoCount] = {
        cdiOffset,
        static_cast<double>(fromTo),
        -gsOffset,
        -gsOffset
    };

    for (int servoId = 0; servoId < servoCount; servoId++)
    {
        moveServo(static_cast<ServoId>(servoId), servoDegree[servoId]);
    }
    return success;
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

    // DEBUGLOG_PRINT(" Current Pos: ");
    // DEBUGLOG_PRINT(axes[axis]->getPosition());
    // DEBUGLOG_PRINT(" Target Pos: ");
    // DEBUGLOG_PRINT(targetPosition);
    // DEBUGLOG_PRINT(" dist: ");
    // DEBUGLOG_PRINT(stepsToMove);
    // DEBUGLOG_PRINT(" RPM: ");
    // DEBUGLOG_PRINTLN(axes[axis]->getRpm());

    moveSteps(axis, stepsToMove);
    return success;
}

HSI::HSIDriveResult HSI::moveServo(ServoId id, double degree)
{
    if (id >= ServoId::servoCount)
    {
        return invalidId;
    }
    double clampedDegree = max(kServoMinimumDegree[id], min(kServoMaximumDegree[id], degree));
    double adjustedDegree = clampedDegree * kServoAdjustRatio[id] + kServoAdjustDegree[id];
    DEBUGLOG_PRINTLN(String(F("Move ")) + servoName(id) + String(F(" to ")) + String(degree) + String(F(" adjusted to ")) + String(adjustedDegree));

    servos[id]->write(adjustedDegree);
    return success;
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

void HSI::resetPositionAndBacklash(HSIAxis axis)
{
    for (uint32_t n = 0; n < axes[axis]->getBacklashSteps(); n++)
    {
        axes[axis]->step(axes[axis]->lastDirectionIsClockwise());
        sendMotorData();
    }
    axes[axis]->resetPosition();
}

HSI::HSIDriveResult HSI::homeAxis(HSIAxis axis)
{
    resetPositionAndBacklash(axis);

    axes[axis]->setRpm(kRpmLimits[axis][maxRpm]);

    // Move until zeroState is *not* triggered
    DEBUGLOG_PRINTLN(String(F("- Leave zero state")));
    if (lookForZeroChange(axis, 370, false) != success)
    {
        return homingError;
    }

    // Move until zeroState *is* triggered
    DEBUGLOG_PRINTLN(String(F("- Search zero state")));
    if (lookForZeroChange(axis, 370, true) != success)
    {
        return homingError;
    }

    axes[axis]->setRpm(kRpmLimits[axis][minRpm]);

    moveDegree(axis, 1, true);

    DEBUGLOG_PRINTLN(String(F("- Slow search end of zero state")));
    if (lookForZeroChange(axis, 60, false) != success)
    {
        return homingError;
    }
    int32_t zeroEndPosition = axes[axis]->getPosition();
    moveDegree(axis, 1, true);

    DEBUGLOG_PRINTLN(String(F("- Slow search start of zero state")));
    if (lookForZeroChange(axis, -65, true) != success)
    {
        return homingError;
    }
    if (lookForZeroChange(axis, -65, false) != success)
    {
        return homingError;
    }
    int32_t zeroStartPosition = axes[axis]->getPosition();

    axes[axis]->setRpm(kRpmLimits[axis][maxRpm]);

    // The real zero is in the middle of the zeroStart and zeroEnd
    int32_t zeroAdjust = (zeroEndPosition - zeroStartPosition) % static_cast<int32_t>(axes[axis]->getTotalSteps()) / 2L;

    DEBUGLOG_PRINT(String(F("- Zero adjust: ")));
    DEBUGLOG_PRINT(zeroAdjust);
    DEBUGLOG_PRINT(String(F(" - Start: ")));
    DEBUGLOG_PRINT(zeroStartPosition);
    DEBUGLOG_PRINT(String(F(" - End: ")));
    DEBUGLOG_PRINTLN(zeroEndPosition);
    moveSteps(axis, zeroAdjust, true);

    moveDegree(axis, kZeroAdjustDegree[axis], true);
    axes[axis]->resetPosition();
    homingState[axis] = homed;
    return success;
}

HSI::HSIDriveResult HSI::lookForZeroChange(HSIAxis axis, int32_t degree, bool targetZeroedState)
{
    DEBUGLOG_PRINT(String(F("    look4Zero on ")) + axisName(axis));
    DEBUGLOG_PRINT(String(F(" - target: ")));
    DEBUGLOG_PRINT(String(targetZeroedState ? F("true") : F("false")));
    DEBUGLOG_PRINT(String(F(" - current: ")));
    DEBUGLOG_PRINTLN(String(zeroedState[axis] ? F("true") : F("false")));

    fetchZeroedState(axis);
    moveDegree(axis, degree);
    while (zeroedState[axis] != targetZeroedState && axes[axis]->getStepsLeft() != 0)
    {
        axes[axis]->run();
        sendMotorData();
        fetchZeroedState(axis);
    }

    if (zeroedState[axis] != targetZeroedState)
    {
        DEBUGLOG_PRINTLN(String(F("   *** Timeout")));
        return homingError;
    }

    DEBUGLOG_PRINTLN(String(F("    reached: ")) + String(zeroedState[axis] ? String(F("true")) : String(F("false"))));
    return success;
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
            int32_t zeroAdjust = static_cast<int32_t>(((zeroEndPosition[axis] - zeroStartPosition) % axes[axis]->getTotalSteps()) / 2L);
            DEBUGLOG_PRINTLN(String(axisName(axis)) + String(F("- Move to true zero position, zero adjust steps: ")) + String(zeroAdjust));
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
        DEBUGLOG_PRINTLN(String(axisName(axis)) + String(F("- Move to adjusted zero position, adjustment degree: ")) + String(kZeroAdjustDegree[axis]));
        axes[axis]->resetPosition();
        moveDegree(axis, kZeroAdjustDegree[axis]);
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

void HSI::fetchZeroedState(HSIAxis axis)
{
    bool state = digitalRead(kHallPins[axis]) == LOW;
    if (state != zeroedState[axis])
    {
        zeroedState[axis] = state;
        DEBUGLOG_PRINTLN(String(F("    > Zeroed state changed on ")) + axisName(axis) + String(F(" axis: ")) + String(state ? F("true") : F("false")));
    }
    // return state;
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
