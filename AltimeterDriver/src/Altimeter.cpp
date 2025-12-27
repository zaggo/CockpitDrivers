#include <Altimeter.h>
#include <Wire.h>
#include "DebugLog.h"

Altimeter *Altimeter::instance = nullptr;

Altimeter::Altimeter()
{
    // Setup zeroing pins & interrupts
    Wire.begin();

    mcp = new MCP23017(kMCP23017Address);
    mcp->init();
    mcp->portMode(MCP23017Port::A, 0); // Port A as output
    mcp->portMode(MCP23017Port::B, 0); // Port B as output

    mcp->writeRegister(MCP23017Register::GPIO_A, 0x00); // Reset port A
    mcp->writeRegister(MCP23017Register::GPIO_B, 0x00); // Reset port B

    DEBUGLOG_PRINTLN("MCP23017 initialized");

    axes[hundred] = new CheapStepper(&hundredsPattern);
    axes[thousand] = new CheapStepper(&thousandsPattern, true);
    axes[tenshousand] = new CheapStepper(6, 7, 12, 13);

    for (int axis = 0; axis < altimeterAxisCount; axis++)
    {
        DEBUGLOG_PRINTLN(String("Initializing ") + axisName(static_cast<AltimeterAxis>(axis)));
        DEBUGLOG_PRINT(String("Total Steps: ") + kTotalSteps[axis]);
        DEBUGLOG_PRINTLN(String(", MaxRPM: ") + static_cast<int>(kRpmLimits[axis][maxRpm]));

        axes[axis]->setTotalSteps(kTotalSteps[axis]);
        axes[axis]->setRpm(kRpmLimits[axis][maxRpm]);
        pinMode(kHallPins[axis], INPUT_PULLUP);
        homingState[axis] = unknown;
    }

    for (int servoId = 0; servoId < servoCount; servoId++)
    {
        // DEBUGLOG_PRINTLN(String("Initializing ") + servoName(static_cast<ServoId>(servoId)));
        // DEBUGLOG_PRINTLN(String("Servo Pin: ") + static_cast<int>(kServoPins[servoId]));
        servos[servoId] = new Servo();
        servos[servoId]->attach(kServoPins[servoId]);
    }

    pinMode(kPotentiometerPin, INPUT);

    delay(2000); // Wait for the Pullup resistors to stabilize

    for (int axis = 0; axis < altimeterAxisCount; axis++)
    {
        zeroedState[axis] = digitalRead(kHallPins[axis]) == LOW;
    }

    instance = this;
    DEBUGLOG_PRINTLN(String(F("Altimeter setup complete")));
}

Altimeter::~Altimeter()
{
    for (int servoId = 0; servoId < servoCount; servoId++)
    {
        servos[servoId]->detach();
        delete servos[servoId];
    }
    for (int axis = 0; axis < altimeterAxisCount; axis++)
    {
        axes[axis]->off();
        delete axes[axis];
    }
    mcp->writeRegister(MCP23017Register::GPIO_A, 0x00); // Reset port A
    mcp->writeRegister(MCP23017Register::GPIO_B, 0x00); // Reset port B
    delete mcp;
}

void Altimeter::loop()
{
    if (instance == NULL)
    {
        return;
    }

    uint32_t uS = micros();
#if COUPLED_MODE
    if (axesCoupled && axes[hundred]->getStepsLeft() != 0)
    {
        axes[hundred]->run(uS);
        sendMotorData();

        for (int axisIndex = 1; axisIndex < altimeterAxisCount; axisIndex++)
        {
            AltimeterAxis axis = static_cast<AltimeterAxis>(axisIndex);
            int32_t targetPosition = static_cast<int32_t>(axes[axisIndex - 1]->getPosition() / 10L);
            int32_t stepsToMove = (targetPosition - axes[axis]->getPosition() + axes[axis]->getStepsLeft());
            if (abs(stepsToMove) > 5)
            {
                moveSteps(axis, stepsToMove, true);
            }
        }
    }
    else
#endif
    {
        for (int axis = 0; axis < altimeterAxisCount; axis++)
        {
            axes[axis]->run(uS);
        }
        sendMotorData();
    }

    // double heightInFeet = currentHeightInFeet();
    // if (currentFlagState == Flag::off && heightInFeet < 10000.0)
    // {
    //     setFlag(on);
    // }
    // else if (currentFlagState == Flag::on && heightInFeet >= 10100.0)
    // {
    //     setFlag(off);
    // }
}

float Altimeter::fetchPressureRatio()
{
    int potValue = analogRead(kPotentiometerPin);
    // DEBUGLOG_PRINTLN(String(F("Raw Pot Value: ")) + String(potValue));
    float ratio = static_cast<float>(potValue - kZeroPressure) / static_cast<float>(kHundredPercentPressure - kZeroPressure);
    return roundf(500.f * constrain(ratio, 0.0f, 1.0f)) / 500.f;
}

void Altimeter::sendMotorData()
{
    uint8_t portA = thousandsPattern << 4 | (hundredsPattern & 0x0F); // Combine the patterns for port A
    if (currentPortA != portA)
    {
        currentPortA = portA;
        // DEBUGLOG_PRINTLN(String(F("Sent portA: 0b")) + String(portA, BIN));
        mcp->writePort(MCP23017Port::A, portA);
    }
    // uint8_t portB = compPattern & 0x0F; // Combine the patterns for port B
    // if (currentPortB != portB)
    // {
    //     currentPortB = portB;
    //     mcp->writePort(MCP23017Port::B, portB);
    // }
}

void Altimeter::stopAllAxes()
{
    for (int axis = 0; axis < altimeterAxisCount; axis++)
    {
        axes[axis]->stop();
    }
}

void Altimeter::offAllAxes()
{
    for (int axis = 0; axis < altimeterAxisCount; axis++)
    {
        axes[axis]->off();
    }
}

Altimeter::AltimeterDriveResult Altimeter::lookForZeroChangeNonBlocking(AltimeterAxis axis, bool targetZeroedState)
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

Altimeter::AltimeterDriveResult Altimeter::nextHomingState(AltimeterAxis axis)
{
    Altimeter::AltimeterDriveResult zeroState;
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

Altimeter::AltimeterDriveResult Altimeter::homeAllAxis()
{
    stopAllAxes();
    axesCoupled = false;

    for (int axisIndex = 0; axisIndex < altimeterAxisCount; axisIndex++)
    {
        homingState[axisIndex] = unknown;
        nextHomingState(static_cast<AltimeterAxis>(axisIndex));
    }

    while (!checkAllHomed())
    {
        for (int axisIndex = 0; axisIndex < altimeterAxisCount; axisIndex++)
        {
            AltimeterAxis axis = static_cast<AltimeterAxis>(axisIndex);
            AltimeterDriveResult result = nextHomingState(axis);
            if (result != success)
            {
                DEBUGLOG_PRINTLN(String(F("*** Error homing '")) + errorName(result) + String(F("' on axis ")) + axisName(axis));
                return result;
            }
            axes[axisIndex]->run();
        }
        sendMotorData();
    }

    moveServo(flagServo, kServoMaximumDegree[flagServo]); // Move flag down after homing

    isHomed = true;
#if COUPLED_MODE
    axesCoupled = true;
#endif
    return success;
}

bool Altimeter::checkAllHomed()
{
    for (int i = 0; i < altimeterAxisCount; i++)
    {
        if (homingState[i] != homed)
        {
            return false;
        }
    }
    return true;
}

double Altimeter::currentHeightInFeet()
{
#if COUPLED_MODE
    return (static_cast<double>(axes[hundred]->getPosition()) / static_cast<double>(axes[hundred]->getTotalSteps())) * 1000.0;
#else
    double heightinFeet = 0.0;
    double scale = 1000.0;
    for (int axisIndex = 0; axisIndex < altimeterAxisCount; axisIndex++)
    {
        AltimeterAxis axis = static_cast<AltimeterAxis>(axisIndex);
        heightinFeet += (static_cast<double>(axes[axis]->getPosition()) / static_cast<double>(axes[axis]->getTotalSteps())) * scale;
        scale *= 10.0;
    }
    return heightinFeet;
#endif
}

Altimeter::AltimeterDriveResult Altimeter::moveToHeight(double heightInFeet)
{
#if COUPLED_MODE
    if (!axesCoupled)
    {
        DEBUGLOG_PRINTLN(String(F(">> Altimeter axes not coupled, cannot move to height")));
        return coupledAxisError;
    }
    // double heightDifference = heightInFeet - currentHeightInFeet();
    // if (abs(heightDifference) > 5000.0) {
    //     return skipToHeight(heightInFeet);
    // }

    int32_t targetPosition = static_cast<int32_t>((heightInFeet / 1000.0) * static_cast<double>(axes[hundred]->getTotalSteps()));
    int32_t stepsToMove = targetPosition - axes[hundred]->getPosition();
    DEBUGLOG_PRINTLN(String(F("Moving to height: ")) + String(heightInFeet) + String(F(" feet, Steps to move: ")) + String(stepsToMove) + String(F(", Target Position: ")) + String(targetPosition) + String(F(", Current Position: ")) + String(axes[hundred]->getPosition()));
    moveSteps(hundred, stepsToMove);
    DEBUGLOG_PRINTLN(String(F("Steps left: ")) + String(axes[hundred]->getStepsLeft()));
    return success;
#else
    // Calculate the position of each axis based on the desired height in 0-350 degrees
    double hundredsDegree = fmod(heightInFeet, 1000.0) * 360.0 / 1000.0;
    double thousandsDegree = fmod(heightInFeet / 1000.0, 10.0) * 360.0 / 10.0;
    double tenThousandsDegree = fmod(heightInFeet / 10000.0, 10.0) * 360.0 / 10.0;
    double flagDegree = kServoMaximumDegree[flagServo];
    if (heightInFeet > 9000.0 && heightInFeet <10000.0) {
        flagDegree = kServoMinimumDegree[flagServo] + (kServoMaximumDegree[flagServo] - kServoMinimumDegree[flagServo]) * ((10000.0 - heightInFeet) / 1000.0);
    } else if (heightInFeet >= 10000.0) {
        flagDegree = kServoMinimumDegree[flagServo];
    }
    DEBUGLOG_PRINTLN(String(F("Moving to height: ")) + String(heightInFeet) + String(F(" feet -> Degrees: [")) + String(hundredsDegree) + String(F(", ")) + String(thousandsDegree) + String(F(", ")) + String(tenThousandsDegree) + String(F(", Flag: ")) + String(flagDegree) + String(F("]")));
    return moveToDegree(hundredsDegree, thousandsDegree, tenThousandsDegree, flagDegree);
#endif
}

Altimeter::AltimeterDriveResult Altimeter::moveToDegree(double hundredDegree, double thousandDegree, double tenThousandDegree, double flagDegree)
{
    if (axesCoupled)
    {
        DEBUGLOG_PRINTLN(String(F(">> Altimeter axes coupled, cannot move to degree individually")));
        return coupledAxisError;
    }

    const double motorDegree[altimeterAxisCount] = {
        hundredDegree,
        thousandDegree,
        tenThousandDegree};

    for (int axis = 0; axis < altimeterAxisCount; axis++)
    {
        moveToDegree(static_cast<AltimeterAxis>(axis), motorDegree[axis]);
    }

    moveServo(flagServo, flagDegree);
    return success;
}

Altimeter::AltimeterDriveResult Altimeter::moveToDegree(AltimeterAxis axis, double degree)
{
    // DEBUGLOG_PRINT("MoveToDegree: ");
    // DEBUGLOG_PRINT(axisName(axis));
    if (!isHomed)
    {
        DEBUGLOG_PRINTLN(String(F(" >> Not Homed")));
        return AltimeterDriveResult::notHomed;
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

Altimeter::AltimeterDriveResult Altimeter::moveServo(ServoId id, double degree, bool calibration)
{
    if (id >= ServoId::servoCount)
    {
        return invalidId;
    }
    double adjustedDegree;
    if (calibration)
    {
        adjustedDegree = degree;
        DEBUGLOG_PRINTLN(String(F("Calibrate ")) + servoName(id) + String(F(" to ")) + String(degree));
    }
    else
    {
        double clampedDegree = max(kServoMinimumDegree[id], min(kServoMaximumDegree[id], degree));
        adjustedDegree = clampedDegree + kServoAdjustDegree[id];
        if (id == flagServo)
        {
            if (currentFlagDegree == adjustedDegree)
            {
                return success;
            }
            currentFlagDegree = adjustedDegree;
        }
        // DEBUGLOG_PRINTLN(String(F("Move ")) + servoName(id) + String(F(" to ")) + String(degree) + String(F(" adjusted to ")) + String(adjustedDegree));
    }
    servos[id]->write(adjustedDegree);
    return success;
}

void Altimeter::moveSteps(AltimeterAxis axis, int32_t steps, bool synchron)
{
    axes[axis]->newMove(steps > 0, static_cast<uint32_t>(abs(steps)));
    while (synchron && axes[axis]->getStepsLeft() != 0)
    {
        axes[axis]->run();
        sendMotorData();
    }
}

void Altimeter::moveDegree(AltimeterAxis axis, int32_t degree, bool synchron)
{
    int32_t numSteps = static_cast<int32_t>(degree) * static_cast<int32_t>(axes[axis]->getTotalSteps()) / DEGREE_FULL_ROTATION;
    moveSteps(axis, numSteps, synchron);
}

int32_t Altimeter::calculateShortestPath(AltimeterAxis axis, int32_t targetPosition)
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

uint32_t Altimeter::normalizePosition(int32_t position, uint32_t totalSteps)
{
    return static_cast<uint32_t>((position % totalSteps + totalSteps) % totalSteps);
}

Altimeter::AltimeterDriveResult Altimeter::homeAxis(AltimeterAxis axis)
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
    if (lookForZeroChange(axis, 60, false) != success)
    {
        return homingError;
    }
    int32_t zeroEndPosition = axes[axis]->getPosition();

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

    return success;
}

Altimeter::AltimeterDriveResult Altimeter::lookForZeroChange(AltimeterAxis axis, int32_t degree, bool targetZeroedState)
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

void Altimeter::fetchZeroedState(AltimeterAxis axis)
{
    bool state = digitalRead(kHallPins[axis]) == LOW;
    if (state != zeroedState[axis])
    {
        zeroedState[axis] = state;
        // DEBUGLOG_PRINTLN(String(F("    > Zeroed state changed on ")) + axisName(axis) + String(F(" axis: ")) + String(state ? F("true") : F("false")));
    }
    // return state;
}

String Altimeter::axisName(AltimeterAxis axis)
{
    switch (axis)
    {
    case hundred:
        return "100s";
    case thousand:
        return "1000s";
    case tenshousand:
        return "10ks";
    default:
        return "Unknown";
    }
}

String Altimeter::servoName(ServoId id)
{
    switch (id)
    {
    case flagServo:
        return "Flag Servo";
    default:
        return "Unknown";
    }
}

String Altimeter::errorName(AltimeterDriveResult result)
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
    case coupledAxisError:
        return "Coupled Axis Error";
    case homingTimeout:
        return "Homing Timeout";
    default:
        return "Unknown Error";
    }
}