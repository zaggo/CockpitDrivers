#include <HSI.h>
#include <Wire.h>
#include "DebugLog.h"

HSI *HSI::instance = nullptr;
ISR(PCINT1_vect) // Comp Interrupt
{
    if (HSI::instance)
    {
        bool buttonState = digitalRead(kCompEncoderPushButtonPin) == LOW;
        bool aPinState = digitalRead(kCompEncoderAPin) == LOW;
        bool bPinState = digitalRead(kCompEncoderBPin) == LOW;
       //DEBUGLOG_PRINTLN(" [PCINT1_vect (a|b|p): " + String(aPinState ? "T|":"F|") + String(bPinState ? "T|":"F|") + String(buttonState ? "T":"F") + "] ");

        if (HSI::instance->currentCompEncoderButtonState != buttonState)
        {
            HSI::instance->currentCompEncoderButtonState = buttonState;
        }

        uint8_t s = HSI::instance->compEncoderState & 0b0011;
		if (aPinState) s |= 0b0100;
		if (bPinState) s |= 0b1000;
        HSI::instance->compEncoderState = (s >> 2);
        switch (s) {
            case 0b0000: case 0b0101: case 0b1010: case 0b1111:
                break;
            case 0b0001: case 0b0111: case 0b1000: case 0b1110:
                HSI::instance->currentCompEncoder++; break;
            case 0b0010: case 0b0100: case 0b1011: case 0b1101:
                HSI::instance->currentCompEncoder--; break;
            case 0b0011: case 0b1100:
                HSI::instance->currentCompEncoder += 2; break;
            default:
                HSI::instance->currentCompEncoder -= 2; break;
        }
       //DEBUGLOG_PRINTLN(" [INT currentCompEncoder:" + String(HSI::instance->currentCompEncoder) + "] ");
    }
}
ISR(PCINT2_vect) // CDI Interrupt
{
    if (HSI::instance)
    {
        bool buttonState = digitalRead(kCDIEncoderPushButtonPin) == LOW;
        bool aPinState = digitalRead(kCDIEncoderAPin) == LOW;
        bool bPinState = digitalRead(kCDIEncoderBPin) == LOW;
        //DEBUGLOG_PRINTLN(" [PCINT2_vect (a|b|p): " + String(aPinState ? "T|":"F|") + String(bPinState ? "T|":"F|") + String(buttonState ? "T":"F") + "] ");
        if (HSI::instance->currentCdiEncoderButtonState != buttonState)
        {
            HSI::instance->currentCdiEncoderButtonState = buttonState;
        }

        uint8_t s = HSI::instance->cdiEncoderState & 0b0011;
		if (aPinState) s |= 0b0100;
		if (bPinState) s |= 0b1000;
        HSI::instance->cdiEncoderState = (s >> 2);
        switch (s) {
            case 0b0000: case 0b0101: case 0b1010: case 0b1111:
                break;
            case 0b0001: case 0b0111: case 0b1000: case 0b1110:
                HSI::instance->currentCdiEncoder++; break;
            case 0b0010: case 0b0100: case 0b1011: case 0b1101:
                HSI::instance->currentCdiEncoder--; break;
            case 0b0011: case 0b1100:
                HSI::instance->currentCdiEncoder += 2; break;
            default:
                HSI::instance->currentCdiEncoder -= 2; break;
        }
        //DEBUGLOG_PRINTLN(" [INT currentCdiEncoder:" + String(HSI::instance->currentCdiEncoder) + "] ");
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

    axes[compass] = new CheapStepper(&compPattern);
    axes[cdi] = new CheapStepper(&cdiPattern);
    axes[hdg] = new CheapStepper(&hdgPattern);

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
    pinMode(kCDIEncoderAPin, INPUT_PULLUP);
    pinMode(kCDIEncoderBPin, INPUT_PULLUP);
    pinMode(kCDIEncoderPushButtonPin, INPUT_PULLUP);
    pinMode(kCompEncoderAPin, INPUT_PULLUP);
    pinMode(kCompEncoderBPin, INPUT_PULLUP);
    pinMode(kCompEncoderPushButtonPin, INPUT_PULLUP);

    delay(2000); // Wait for the Pullup resistors to stabilize
    cdiEncoderButtonState = digitalRead(kCDIEncoderPushButtonPin) == LOW;
    currentCdiEncoderButtonState = cdiEncoderButtonState;
    cdiEncoderState = 0;
    if(digitalRead(kCDIEncoderAPin)) cdiEncoderState |= 1;
    if(digitalRead(kCDIEncoderBPin)) cdiEncoderState |= 2;
    compEncoderState = 0;
    if(digitalRead(kCompEncoderAPin)) compEncoderState |= 1;
    if(digitalRead(kCompEncoderBPin)) compEncoderState |= 2;
    compEncoderButtonState = digitalRead(kCompEncoderPushButtonPin) == LOW;
    currentCompEncoderButtonState = compEncoderButtonState;
    for (int axis = 0; axis < hsiAxisCount; axis++) {
        zeroedState[axis] = digitalRead(kHallPins[axis]) == LOW;
    }

    PCICR = (1 << PCIE2) | (1 << PCIE1);
    PCMSK1 = kCompEncoderInterruptMask;
    PCMSK2 = kCDIEncoderInterruptMask;

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

    if (compEncoder != currentCompEncoder)
    {
        compEncoder = currentCompEncoder;
        //DEBUGLOG_PRINTLN(" [compEncoder:" + String(compEncoder) + "] ");
    }
    if (cdiEncoder != currentCdiEncoder)
    {
        cdiEncoder = currentCdiEncoder;
        //DEBUGLOG_PRINTLN(" [cdiEncoder:" + String(cdiEncoder) + "] ");
    }
    const uint32_t debounceDelay = 50000; // microseconds

    // Debouncing the CDI encoder push button state
    if (currentCdiEncoderButtonState != cdiEncoderButtonState && (uS - lastCdiDebounceTime) > debounceDelay)
    {
        lastCdiDebounceTime = uS;
        cdiEncoderButtonState = currentCdiEncoderButtonState;
        //DEBUGLOG_PRINTLN(" [cdiEncoderButtonState:" + String(cdiEncoderButtonState) + "] ");
    }

    // Debouncing the Compass encoder push button state
    if (currentCompEncoderButtonState != compEncoderButtonState && (uS - lastCompDebounceTime) > debounceDelay)
    {
        lastCompDebounceTime = uS;
        compEncoderButtonState = currentCompEncoderButtonState;
        //DEBUGLOG_PRINTLN(" [compEncoderButtonState:" + String(compEncoderButtonState) + "] ");
    }
}

void HSI::sendMotorData()
{
    uint8_t portA = hdgPattern << 4 | (cdiPattern & 0x0F); // Combine the patterns for port A
    if (currentPortA != portA)
    {
        currentPortA = portA;
        mcp->writePort(MCP23017Port::A, portA);
    }
    uint8_t portB = compPattern & 0x0F; // Combine the patterns for port B
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
    stopAllAxes();    
    isHomed = false;
    for (int i = 0; i < hsiAxisCount; i++)
    {
        HSIAxis axis = static_cast<HSIAxis>(i);
        DEBUGLOG_PRINTLN(String(F("Start homing ")) + axisName(axis) + String(F(" axis")));
        if (homeAxis(axis) != success)
        {
            DEBUGLOG_PRINTLN(String(F("*** Error homing ")) + axisName(axis) + String(F(" axis")));
            // return homingError;
        }
        else
        {
            DEBUGLOG_PRINTLN(String(F("End homing ")) + axisName(axis) + String(F(" axis")));
        }
    }

    isHomed = true;
    return success;
}

HSI::HSIDriveResult HSI::moveToDegree(double cdiDegree, double compassDegree, double hdgDegree, double vorOffset, FromTo fromTo, double vsiOffset)
{
    const double motorDegree[hsiAxisCount] = {
        fmod(compassDegree- cdiDegree + 360.0, 360.0),
        compassDegree,
        fmod(compassDegree - hdgDegree + 360.0, 360.0)
    };

    for (int axis = 0; axis < hsiAxisCount; axis++)
    {
        moveToDegree(static_cast<HSIAxis>(axis), motorDegree[axis]);
    }

    const double servoDegree[servoCount] = {
        vorOffset,
        static_cast<double>(fromTo), 
        -vsiOffset,
        -vsiOffset
    };

    for (int servoId = 0; servoId < servoCount; servoId++)
    {
        moveServo(static_cast<ServoId>(servoId), servoDegree[servoId]);
    }
    return success;
}

HSI::HSIDriveResult HSI::moveToDegree(HSIAxis axis, double degree)
{
    // DEBUGLOG_PRINT("MoveToDegree: ");
    // DEBUGLOG_PRINT(axisName(axis));
    if (!isHomed)
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

HSI::HSIDriveResult HSI::homeAxis(HSIAxis axis)
{
    axes[axis]->resetPosition();
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

    DEBUGLOG_PRINTLN(String(F("- Slow search end of zero state")));
    if (lookForZeroChange(axis, 30, false) != success)
    {
        return homingError;
    }
    uint32_t zeroEndPosition = axes[axis]->getPosition();

    DEBUGLOG_PRINTLN(String(F("- Slow search start of zero state")));
    if (lookForZeroChange(axis, -35, true) != success)
    {
        return homingError;
    }
    if (lookForZeroChange(axis, -35, false) != success)
    {
        return homingError;
    }
    uint32_t zeroStartPosition = axes[axis]->getPosition();

    axes[axis]->setRpm(kRpmLimits[axis][maxRpm]);

    // The real zero is in the middle of the zeroStart and zeroEnd
    int32_t zeroAdjust = static_cast<int32_t>(((zeroEndPosition - zeroStartPosition) % axes[axis]->getTotalSteps()) / 2L);

    DEBUGLOG_PRINT(String(F("- Zero adjust: ")));
    DEBUGLOG_PRINT(zeroAdjust);
    DEBUGLOG_PRINT(String(F(" - Start: ")));
    DEBUGLOG_PRINT(zeroStartPosition);
    DEBUGLOG_PRINT(String(F(" - End: ")));
    DEBUGLOG_PRINTLN(zeroEndPosition);
    moveSteps(axis, zeroAdjust, true);

    moveDegree(axis, kZeroAdjustDegree[axis], true);
    axes[axis]->resetPosition();

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
    case cdi:
        return "CDI";
    case compass:
        return "Compass";
    case hdg:
        return "Hdg";
    default:
        return "Unknown";
    }
}

String HSI::servoName(ServoId id)
{
    switch (id)
    {
    case vorServo:
        return "VOR Servo";
    case fromToServo:
        return "From-To Servo";
    case vsi1Servo:
        return "VSI1 Servo";
    case vsi2Servo:
        return "VSI2 Servo";
    default:
        return "Unknown";
    }
}