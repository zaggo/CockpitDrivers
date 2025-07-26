#include <Arduino.h>
#include "HSI.h"
#include "Configuration.h"
#include "DebugLog.h"

HSI *HSI::instance = nullptr;

// Constructor & Destructor
HSI::HSI()
{
    // Setup zeroing pins & interrupts
    instance = this;

    // Initialize servos
    servos[vorServo] = new Servo();
    servos[vorServo]->attach(kVorServoPin); 
    servos[fromToServo] = new Servo();
    servos[fromToServo]->attach(kFromToServoPin);
    servos[gsServo] = new Servo();
    servos[gsServo]->attach(kGSServoPin);

    // Initialize steppers   
    for (int i = 0; i < activeAxesCount; i++) {
        HSIAxis axis = activeAxes[i];

        pinMode(kMotorPins[axis][kZeroedPin], INPUT_PULLUP);
        // zeroedState[axis] = !digitalRead(kMotorPins[axis][kZeroedPin]);

        // switch (axis) {
        //     case vor:
        //     attachInterrupt(digitalPinToInterrupt(kMotorPins[axis][kZeroedPin]), ISR_VorPinChange, CHANGE);
        //     break;
        //     case compass:
        //     attachInterrupt(digitalPinToInterrupt(kMotorPins[axis][kZeroedPin]), ISR_CompassPinChange, CHANGE);
        //     break;
        //     case bug:
        //     attachInterrupt(digitalPinToInterrupt(kMotorPins[axis][kZeroedPin]), ISR_BugPinChange, CHANGE);
        //     break;
        //     default:
        //     break;
        // }

        initializeStepper(axis);
    }
}

HSI::~HSI()
{
    delete servos[vorServo];
    delete servos[fromToServo];
    delete servos[gsServo];

    for (int i = 0; i < activeAxesCount; i++) {
        HSIAxis axis = activeAxes[i];
        // detachInterrupt(digitalPinToInterrupt(kMotorPins[axis][kZeroedPin]));
        delete axes[axis];
    }
}

// Private method to initialize steppers
void HSI::initializeStepper(HSIAxis axis)
{
    DEBUGLOG_PRINTLN("Initializing " + axisName(axis) + " with pins ["+kMotorPins[axis][kPin1]+", "+kMotorPins[axis][kPin2]+", "+kMotorPins[axis][kPin3]+", "+kMotorPins[axis][kPin4]+"]");
    DEBUGLOG_PRINT("Total Steps: " + kTotalSteps[axis] ); 
    DEBUGLOG_PRINTLN(", MaxRPM: " + static_cast<int>(kRpmLimits[axis][kMaxRpm]));

    axes[axis] = new CheapStepper(
        kMotorPins[axis][kPin1], 
        kMotorPins[axis][kPin2], 
        kMotorPins[axis][kPin3], 
        kMotorPins[axis][kPin4]
        );

    axes[axis]->setTotalSteps(kTotalSteps[axis]);
    axes[axis]->setRpm(kRpmLimits[axis][kMaxRpm]);
}

// Interrupt Service Routines  
// void HSI::ISR_VorPinChange()
// {
//     if (instance)
//     {
//         instance->zeroedState[HSIAxis::vor] = !digitalRead(kMotorPins[HSIAxis::vor][kZeroedPin]);
//     }
// }

// void HSI::ISR_CompassPinChange()
// {
//     if (instance)
//     {
//         instance->zeroedState[HSIAxis::compass] = !digitalRead(kMotorPins[HSIAxis::compass][kZeroedPin]);
//     }
// }

// void HSI::ISR_BugPinChange()
// {
//     if (instance)
//     {
//         instance->zeroedState[HSIAxis::bug] = !digitalRead(kMotorPins[HSIAxis::bug][kZeroedPin]);
//     }
// }

// Public Methods
/**
 * @brief Homes all axes of the HSI.
 *
 * This function attempts to home the VOR, Compass and Bug axes of the HSI.
 *
 * @return GyroDriveResult::success if all axes are homed successfully,
 *         GyroDriveResult::homingError if any axis fails to home.
 */
HSI::HSIDriveResult HSI::homeAllAxis()
{
    stopAllAxes();

    for (int i = 0; i < activeAxesCount; i++) {
        HSIAxis axis = activeAxes[i];
        DEBUGLOG_PRINTLN("Start homing " + axisName(axis) + " axis");
        if (homeAxis(axis) != HSIDriveResult::success)
        {
            DEBUGLOG_PRINTLN("Error homing " + axisName(axis) + " axis");
            return HSIDriveResult::homingError;
        }
        DEBUGLOG_PRINTLN("End homing " + axisName(axis) + " axis");
    }

    isHomed = true;
    return HSIDriveResult::success;
}

/**
 * @brief Moves the HSI to the specified VOR, Compass and Bug degrees.
 *
 * This function calculates the target positions for VOR, Compass and Bug based on the provided degrees,
 * and then moves the HSI to these positions.
 *
 * @param vorDegree The target VOR degree to move to.
 * @param compassDegree The target Compass degree to move to.
 * @param bugDegree The target Bug degree to move to.
 * @param vorOffset The offset to apply to the VOR degree.
 * @param fromTo The FromTo position to move the HSI to.
 * @param gsOffset The offset to apply to the GS degree.
 * @return HSIDriveResult The result of the move operation, indicating success or failure.
 *
 * @note The function will return HSIDriveResult::notHomed if the HSI is not homed.
 */
HSI::HSIDriveResult HSI::moveToDegree(double vorDegree, double compassDegree, double bugDegree, double vorOffset, FromTo fromTo, double gsOffset)
{
    const double motorDegree[HSIAxis::hsiAxisCount] = {vorDegree, compassDegree, bugDegree};

    for (int i = 0; i < activeAxesCount; i++) {
        HSIAxis axis = activeAxes[i];
        DEBUGLOG_PRINT(axisName(axis));
        moveDegree(axis, motorDegree[axis]);
    }

    moveServo(vorServo, vorOffset);
    moveServo(fromToServo, static_cast<double>(fromTo));
    moveServo(gsServo, gsOffset);

    return HSIDriveResult::success;
}

HSI::HSIDriveResult HSI::moveServo(ServoId id, double degree) {
    if (id >= ServoId::servoCount) {
        return HSIDriveResult::invalidId;
    } 

    servos[id]->write(degree);
    return HSIDriveResult::success;
}

HSI::HSIDriveResult HSI::moveToDegree(HSIAxis axis, double degree)
{
    if (!isHomed)
    {
        return HSIDriveResult::notHomed;
    }

    int32_t targetPosition = static_cast<int32_t>(degree * static_cast<double>(axes[axis]->getTotalSteps()) / static_cast<double>(DEGREE_FULL_ROTATION));
    int32_t stepsToMove = calculateShortestPath(axis, targetPosition);

    DEBUGLOG_PRINT("Current Pos: ");
    DEBUGLOG_PRINT(axes[axis]->getPosition());
    DEBUGLOG_PRINT(" Target Pos: ");
    DEBUGLOG_PRINT(targetPosition);
    DEBUGLOG_PRINT(" dist: ");
    DEBUGLOG_PRINT(stepsToMove);
    DEBUGLOG_PRINT(" RPM: ");
    DEBUGLOG_PRINTLN(axes[axis]->getRpm());

    moveSteps(axis, stepsToMove);

    return HSIDriveResult::success;
}

/**
 * @brief Runs all axes of the HSI.
 *
 * This function runs the VOR, Compass and Bug axes of the HSI.
 */
void HSI::runAllAxes()  
{
    for (int i = 0; i < activeAxesCount; i++) 
    {
        axes[activeAxes[i]]->run();
    }
}

/**
 * @brief Stops all axes of the HSI.
 *
 * This function stops the VOR, Compass and Bug axes of the HSI.
 */
void HSI::stopAllAxes()
{
    for (int i = 0; i < activeAxesCount; i++) 
    {
        axes[activeAxes[i]]->stop();
    }
}

/**
 * @brief Turns off all axes of the HSI.
 *
 * This function turns off the VOR, Compass and Bug axes of the HSI.
 */
void HSI::offAllAxes()
{
    for (int i = 0; i < activeAxesCount; i++) 
    {
        axes[activeAxes[i]]->off();
    }
}

// Private Methods
/**
 * @brief Homes the VOR axis of the HSI.
 *
 * This function performs a homing sequence for the VOR axis by moving the axis
 * until the zeroed state is triggered.
 *
 * @return HSIDriveResult::success if the VOR axis is homed successfully,
 *         HSIDriveResult::homingError if the VOR axis fails to home.
 */
HSI::HSIDriveResult HSI::homeAxis(HSIAxis axis)
{
    axes[axis]->resetPosition();
    axes[axis]->setRpm(kRpmLimits[axis][kMaxRpm]);

    // Move until zeroState is *not* triggered
    DEBUGLOG_PRINTLN("Leave zero state");
    if (lookForZeroChange(axis, 370, false) != HSIDriveResult::success)
    {
        return HSIDriveResult::homingError;
    }

    // Move until zeroState *is* triggered
    DEBUGLOG_PRINTLN("Search zero state");
    if (lookForZeroChange(axis, 370, true) != HSIDriveResult::success)
    {
        return HSIDriveResult::homingError;
    }

    axes[axis]->setRpm(kRpmLimits[axis][kMinRpm]);

    DEBUGLOG_PRINTLN("Slow search end of zero state");
    if (lookForZeroChange(axis, 30, false) != HSIDriveResult::success)
    {
        return HSIDriveResult::homingError;
    }
    uint32_t zeroEndPosition = axes[axis]->getPosition();

    DEBUGLOG_PRINTLN("Slow search start of zero state");
    if (lookForZeroChange(axis, -35, true) != HSIDriveResult::success)
    {
        return HSIDriveResult::homingError;
    }
    if (lookForZeroChange(axis, -35, false) != HSIDriveResult::success)
    {
        return HSIDriveResult::homingError;
    }
    uint32_t zeroStartPosition = axes[axis]->getPosition();

    axes[axis]->setRpm(kRpmLimits[axis][kMaxRpm]);

    // The real zero is in the middle of the zeroStart and zeroEnd
    int32_t zeroAdjust = static_cast<int32_t>(((zeroEndPosition - zeroStartPosition) % axes[axis]->getTotalSteps()) / 2L);

    DEBUGLOG_PRINTLN("Zero adjust...");

    moveSteps(axis, zeroAdjust, true);

    moveDegree(axis, kZeroAdjustDegree[axis], true);
    axes[axis]->resetPosition();

    DEBUGLOG_PRINTLN("End homing axis");
    return HSIDriveResult::success;
}

void HSI::moveSteps(HSIAxis axis, int16_t steps, bool synchron)
{
    axes[axis]->newMove(steps > 0, static_cast<uint32_t>(abs(steps)));
    while (synchron && axes[axis]->getStepsLeft() != 0)
    {
        runAllAxes();
    }
}

void HSI::moveDegree(HSIAxis axis, int16_t degree, bool synchron)
{
    int32_t numSteps = static_cast<int32_t>(degree) * static_cast<int32_t>(axes[axis]->getTotalSteps()) / DEGREE_FULL_ROTATION;
    moveSteps(axis, numSteps, synchron);
}

HSI::HSIDriveResult HSI::lookForZeroChange(HSIAxis axis, int32_t degree, bool targetZeroedState)
{
    DEBUGLOG_PRINTLN("look4Z on "+axisName(axis)+" - target:" + targetZeroedState ? "true" : "false");
    moveDegree(axis, degree);
    while (getZeroedState(axis) != targetZeroedState && axes[axis]->getStepsLeft() != 0)
    {
        axes[axis]->run();
    }

    return HSIDriveResult::success;
}

bool HSI::getZeroedState(HSIAxis axis)
{
    bool state = !digitalRead(kMotorPins[axis][kZeroedPin]);
    DEBUGLOG_PRINTLN("getZeroedState on "+axisName(axis)+" = " + state ? "true" : "false");
    return state;
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

/**
 * @brief Normalizes a position to be within the range [0, totalSteps).
 *
 * This function ensures that the given position is wrapped around within the range
 * from 0 to totalSteps-1, effectively handling any overflow or underflow.
 *
 * @param position The position to be normalized.
 * @param totalSteps The total number of steps for a full rotation.
 * @return The normalized position within the range [0, totalSteps).
 */
uint32_t HSI::normalizePosition(int32_t position, uint32_t totalSteps)
{
    return static_cast<uint32_t>((position % totalSteps + totalSteps) % totalSteps);
}

String HSI::axisName(HSIAxis axis) {
    switch (axis) {
        case vor:
            return "VOR";
        case compass:
            return "Compass";
        case bug:
            return "Bug";
        default:
            return "Unknown";
    }
}
