#include "Arduino.h"
#include "GyroDrive.h"
#include "Configuration.h"
#include "DebugLog.h"

GyroDrive *GyroDrive::instance = nullptr;

// Constructor & Destructor
GyroDrive::GyroDrive()
{
    // Setup zeroing pins & interrupts
    instance = this;
    pinMode(kRollZeroedPin, INPUT_PULLUP);
    pinMode(kPitchZeroedPin, INPUT_PULLUP);
    rollZeroedState = !digitalRead(kRollZeroedPin);
    pitchZeroedState = !digitalRead(kPitchZeroedPin);
    attachInterrupt(digitalPinToInterrupt(kRollZeroedPin), ISR_RollPinChange, CHANGE);
    attachInterrupt(digitalPinToInterrupt(kPitchZeroedPin), ISR_PitchPinChange, CHANGE);

    // Initialize steppers
    initializeStepper(roll, kRollPin1, kRollPin2, kRollPin3, kRollPin4, kRollTotalSteps, kRollMaxRpm);
    initializeStepper(pitch, kPitchPin1, kPitchPin2, kPitchPin3, kPitchPin4, kPitchTotalSteps, kPitchMaxRpm);
}

GyroDrive::~GyroDrive()
{
    detachInterrupt(digitalPinToInterrupt(kRollZeroedPin));
    detachInterrupt(digitalPinToInterrupt(kPitchZeroedPin));

    delete axes[roll];
    delete axes[pitch];
}

// Private method to initialize steppers
void GyroDrive::initializeStepper(Axis axisIndex, uint8_t pin1, uint8_t pin2, uint8_t pin3, uint8_t pin4, uint32_t totalSteps, double maxRpm)
{
    axes[axisIndex] = new CheapStepper(pin1, pin2, pin3, pin4);
    axes[axisIndex]->setTotalSteps(totalSteps);
    axes[axisIndex]->setRpm(maxRpm);
}

// Interrupt Service Routines
void GyroDrive::ISR_RollPinChange()
{
    if (instance)
    {
        instance->rollZeroedState = !digitalRead(kRollZeroedPin);
    }
}

void GyroDrive::ISR_PitchPinChange()
{
    if (instance)
    {
        instance->pitchZeroedState = !digitalRead(kPitchZeroedPin);
    }
}

// Public Methods
/**
 * @brief Homes all axes of the GyroDrive.
 *
 * This function attempts to home the Roll and Pitch axes of the GyroDrive.
 *
 * @return GyroDriveResult::success if all axes are homed successfully,
 *         GyroDriveResult::homingError if any axis fails to home.
 */
GyroDrive::GyroDriveResult GyroDrive::homeAllAxis()
{
    if (homeRollAxis() != GyroDriveResult::success)
    {
        DEBUGLOG_PRINTLN("Error homing Roll axis");
        return GyroDriveResult::homingError;
    }
    if (homePitchAxis() != GyroDriveResult::success)
    {
        DEBUGLOG_PRINTLN("Error homing Pitch axis");
        return GyroDriveResult::homingError;
    }
    isHomed = true;
    // axes[roll]->setRpm(8.0);
    return GyroDriveResult::success;
}

void GyroDrive::runAllAxes()
{
    uint32_t currentMicros = micros();
    axes[roll]->run(currentMicros);
    axes[pitch]->run(currentMicros);
}

void GyroDrive::stopAllAxes()
{
    axes[roll]->stop();
    axes[pitch]->stop();
}

void GyroDrive::offAllAxes()
{
    axes[roll]->off();
    axes[pitch]->off();
}

/**
 * @brief Moves the GyroDrive to the specified roll and pitch degrees.
 *
 * This function calculates the target positions for roll and pitch based on the provided degrees,
 * and then moves the GyroDrive to these positions.
 *
 * @param rollDegree The target roll degree to move to.
 * @param pitchDegree The target pitch degree to move to.
 * @return GyroDriveResult The result of the move operation, indicating success or failure.
 *
 * @note The function will return GyroDriveResult::notHomed if the GyroDrive is not homed.
 */
GyroDrive::GyroDriveResult GyroDrive::moveToDegree(int16_t rollDegree, int16_t pitchDegree)
{
    if (!isHomed)
    {
        return GyroDriveResult::notHomed;
    }

    int32_t targetPositionRoll = static_cast<int32_t>(rollDegree) * static_cast<int32_t>(kRollTotalSteps) / DEGREE_FULL_ROTATION;
    int32_t targetPositionPitch = static_cast<int32_t>(pitchDegree) * static_cast<int32_t>(kPitchTotalSteps) / DEGREE_FULL_ROTATION;
    int32_t correctedPositionPitch = targetPositionPitch - convertPosition(roll, axes[roll]->getPosition());
    int32_t stepsToMoveRoll = calculateShortestPath(axes[roll]->getPosition(), targetPositionRoll, kRollTotalSteps);
    int32_t stepsToMovePitch = calculateShortestPath(axes[pitch]->getPosition(), correctedPositionPitch, kPitchTotalSteps);

    DEBUGLOG_PRINT("Roll Current Pos: ");
    DEBUGLOG_PRINT(axes[roll]->getPosition());
    DEBUGLOG_PRINT(" Target Pos: ");
    DEBUGLOG_PRINT(targetPositionRoll);
    DEBUGLOG_PRINT(" dist: ");
    DEBUGLOG_PRINT(stepsToMoveRoll);
    DEBUGLOG_PRINT(" RPM: ");
    DEBUGLOG_PRINTLN(axes[roll]->getRpm());

    DEBUGLOG_PRINT("Pitch Current Pos: ");
    DEBUGLOG_PRINT(axes[pitch]->getPosition());
    DEBUGLOG_PRINT(" Target Pos: ");
    DEBUGLOG_PRINT(targetPositionPitch);
    DEBUGLOG_PRINT(" dist: ");
    DEBUGLOG_PRINT(stepsToMovePitch);
    DEBUGLOG_PRINT(" RPM: ");
    DEBUGLOG_PRINTLN(axes[pitch]->getRpm());

    moveSteps(stepsToMoveRoll, stepsToMovePitch);

    return GyroDriveResult::success;
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
uint32_t GyroDrive::normalizePosition(int32_t position, uint32_t totalSteps)
{
    return static_cast<uint32_t>((position % totalSteps + totalSteps) % totalSteps);
}

/**
 * @brief Calculates the shortest path in steps to move from the current position to the target position.
 *
 * This function determines the shortest path (in steps) to move from the current position to the target position
 * on a circular axis with a given total number of steps. It calculates the clockwise and counterclockwise distances
 * and returns the shortest distance, with positive values indicating clockwise movement and negative values indicating
 * counterclockwise movement.
 *
 * @param currentPosition The current position on the axis in steps.
 * @param targetPosition The target position on the axis in steps.
 * @param totalSteps The total number of steps for a full rotation of the axis.
 * @return The shortest path in steps to move from the current position to the target position.
 */
int32_t GyroDrive::calculateShortestPath(uint32_t currentPosition, int32_t targetPosition, uint32_t totalSteps)
{
    // Normalize targetStep to the range [0, totalSteps)
    uint32_t normalizedTargetPosition = normalizePosition(targetPosition, totalSteps);

    // Berechne die Differenzen in beide Richtungen
    uint32_t diffCW = (normalizedTargetPosition - currentPosition + totalSteps) % totalSteps;  // Clockwise
    uint32_t diffCCW = (currentPosition - normalizedTargetPosition + totalSteps) % totalSteps; // Counterclockwise

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

// Private Methods

/**
 * @brief Converts the position from one axis to another in terms of steps.
 *
 * This function converts the number of steps from the specified axis to the corresponding
 * number of steps in the target axis. The conversion is based on the total number of steps
 * for a full rotation for each axis.
 *
 * @param fromAxis The axis from which the steps are being converted (roll or pitch).
 * @param steps The number of steps in the fromAxis to be converted.
 * @return The converted number of steps in the target axis.
 */
uint32_t GyroDrive::convertPosition(Axis fromAxis, uint32_t steps)
{
    if (kRollTotalSteps == kPitchTotalSteps)
    {
        return steps;
    }
    switch (fromAxis)
    {
    case roll:
        return steps * kPitchTotalSteps / kRollTotalSteps;
    case pitch:
        return steps * kRollTotalSteps / kPitchTotalSteps;
    default:
        // Default case: return steps without conversion as no specific axis is provided
        return steps;
    }
}

/**
 * @brief Moves the gyro drive to a specified degree for roll and pitch.
 *
 * This function calculates the number of steps required to move the roll and pitch axes
 * to the specified degrees and initiates the movement.
 *
 * @param rollDegree The target degree for the roll axis.
 * @param pitchDegree The target degree for the pitch axis.
 */
void GyroDrive::moveDegree(int16_t rollDegree, int16_t pitchDegree, bool inhibitSpeedChange)
{
    int32_t numStepsRoll = (int)(static_cast<int32_t>(rollDegree) * static_cast<int32_t>(kRollTotalSteps) / DEGREE_FULL_ROTATION);
    int32_t numStepsPitch = (int)(static_cast<int32_t>(pitchDegree) * static_cast<int32_t>(kPitchTotalSteps) / DEGREE_FULL_ROTATION);
    moveSteps(numStepsRoll, numStepsPitch, inhibitSpeedChange);
}

/**
 * @brief Moves the gyro drive by a specified number of steps for roll and pitch.
 *
 * This function calculates the corrected pitch steps and adjusts the delay of the faster axis
 * to ensure synchronized movement. It then initiates the movement for both roll and pitch axes.
 *
 * @param numStepsRoll The number of steps to move for the roll axis.
 * @param numStepsPitch The number of steps to move for the pitch axis.
 */
void GyroDrive::moveSteps(int32_t numStepsRoll, int32_t numStepsPitch, bool inhibitSpeedChange)
{
    int32_t correctedPitchSteps = -numStepsRoll + numStepsPitch;

    DEBUGLOG_PRINT("Move steps Roll: ");
    DEBUGLOG_PRINT(numStepsRoll);
    DEBUGLOG_PRINT(" Pitch: ");
    DEBUGLOG_PRINTLN(correctedPitchSteps);

    if (!inhibitSpeedChange)
    {
        // Find out, which axis has to move more steps adjust the delay of the faster axis
        if (abs(numStepsRoll) > abs(correctedPitchSteps))
        {
            uint32_t delay = axes[roll]->setRpm(kRollMaxRpm); // The axis with the higher steps has to move faster
            DEBUGLOG_PRINT("Delay Roll: ");
            DEBUGLOG_PRINT(delay);
            if (correctedPitchSteps != 0)
            {
                uint32_t slowedDelay = delay * abs(numStepsRoll) / abs(correctedPitchSteps);
                axes[pitch]->setDelay(slowedDelay);
                DEBUGLOG_PRINT(" Pitch (shorter/slower): ");
                DEBUGLOG_PRINTLN(slowedDelay);
            }
            else
            {
                axes[pitch]->setDelay(delay);
                DEBUGLOG_PRINT(" Pitch (same): ");
                DEBUGLOG_PRINTLN(delay);
            }
        }
        else
        {
            uint32_t delay = axes[pitch]->setRpm(kPitchMaxRpm); // The axis with the higher steps has to move faster
            DEBUGLOG_PRINT("Delay Pitch: ");
            DEBUGLOG_PRINT(delay);
            if (numStepsRoll != 0)
            {
                uint32_t slowedDelay = delay * abs(correctedPitchSteps) / abs(numStepsRoll);
                axes[roll]->setDelay(slowedDelay);
                DEBUGLOG_PRINT(" Roll (shorter/slower): ");
                DEBUGLOG_PRINTLN(slowedDelay);
            }
            else
            {
                axes[roll]->setDelay(delay);
                DEBUGLOG_PRINT(" Roll (same): ");
                DEBUGLOG_PRINTLN(delay);
            }
        }
    }

    uint32_t currentMicros = micros();
    axes[roll]->newMove(numStepsRoll > 0, static_cast<uint32_t>(abs(numStepsRoll)), currentMicros);
    axes[pitch]->newMove(correctedPitchSteps > 0, static_cast<uint32_t>(abs(correctedPitchSteps)), currentMicros);
}

/**
 * @brief Homes the roll axis of the GyroDrive.
 *
 * This function performs the homing sequence for the roll axis by moving the axis
 * until it reaches the zero state, then adjusting the position to the middle of the
 * zero state range. The function ensures that the axis is correctly zeroed and ready
 * for operation.
 *
 * @return GyroDriveResult::success if the homing process is successful.
 *         GyroDriveResult::homingError if an error occurs during the homing process.
 */
GyroDrive::GyroDriveResult GyroDrive::homeRollAxis()
{
    DEBUGLOG_PRINTLN("Start homing Roll axis");
    stopAllAxes();
    axes[roll]->resetPosition();
    axes[pitch]->resetPosition();
    axes[roll]->setRpm(kRollMaxRpm);
    axes[pitch]->setRpm(kPitchMaxRpm);

    // Move until zeroState is *not* triggered
    DEBUGLOG_PRINTLN("Leave zero state");
    if (lookForZeroChange(roll, 370, false) != GyroDriveResult::success)
    {
        return GyroDriveResult::homingError;
    }

    // Move until zeroState *is* triggered
    DEBUGLOG_PRINTLN("Search zero state");
    if (lookForZeroChange(roll, 370, true) != GyroDriveResult::success)
    {
        return GyroDriveResult::homingError;
    }

    axes[roll]->setRpm(kPitchMinRpm);
    axes[pitch]->setRpm(kPitchMinRpm);

    DEBUGLOG_PRINTLN("Slow search end of zero state");
    if (lookForZeroChange(roll, 30, false, true) != GyroDriveResult::success)
    {
        return GyroDriveResult::homingError;
    }
    uint32_t zeroEndPosition = axes[roll]->getPosition();

    DEBUGLOG_PRINTLN("Slow search start of zero state");
    if (lookForZeroChange(roll, -35, true, true) != GyroDriveResult::success)
    {
        return GyroDriveResult::homingError;
    }
    if (lookForZeroChange(roll, -35, false, true) != GyroDriveResult::success)
    {
        return GyroDriveResult::homingError;
    }
    uint32_t zeroStartPosition = axes[roll]->getPosition();

    axes[roll]->setRpm(kPitchMaxRpm);
    axes[pitch]->setRpm(kPitchMaxRpm);

    // The real zero is in the middle of the zeroStart and zeroEnd
    int32_t zeroAdjust = static_cast<int32_t>(((zeroEndPosition - zeroStartPosition) % kRollTotalSteps) / 2L);

    DEBUGLOG_PRINTLN("Zero adjust...");

    moveSteps(zeroAdjust, 0);
    while (axes[roll]->getStepsLeft() != 0)
    {
        runAllAxes();
    }

    moveDegree(kRollZeroAdjustDegree, 0);
    while (axes[roll]->getStepsLeft() != 0)
    {
        runAllAxes();
    }
    axes[roll]->resetPosition();

    DEBUGLOG_PRINTLN("End homing Roll axis");
    return GyroDriveResult::success;
}

/**
 * @brief Homes the pitch axis of the GyroDrive.
 *
 * This function performs a homing sequence for the pitch axis by moving the axis
 * to find the zero position. It first moves the axis out of the zero state, then
 * searches for the zero position, and finally adjusts to the middle of the zero zone.
 *
 * @return GyroDriveResult::success if the homing sequence is successful.
 * @return GyroDriveResult::homingError if the homing sequence fails.
 */
GyroDrive::GyroDriveResult GyroDrive::homePitchAxis()
{
    DEBUGLOG_PRINTLN("Start homing Pitch axis");

    // Move until zeroState is *not* triggered
    DEBUGLOG_PRINTLN("Leave zero state");
    if (lookForZeroChange(pitch, -30, false) != GyroDriveResult::success)
    {
        return GyroDriveResult::homingError;
    }

    // We are now not in the zero state anymore, so we can search the zero position
    DEBUGLOG_PRINTLN("Search zero position down...");
    if (lookForZeroChange(pitch, 62, true) != GyroDriveResult::success)
    {
        return GyroDriveResult::homingError;
    }

    DEBUGLOG_PRINTLN("Slow search zero position down...");
    moveDegree(0, -10);
    while (axes[pitch]->getStepsLeft() != 0)
    {
        runAllAxes();
    }
    stopAllAxes();

    axes[pitch]->setRpm(kPitchMinRpm); // As slow as possible
    // We are now not in the zero state anymore, so we can search the zero position
    if (lookForZeroChange(pitch, 15, true, true) != GyroDriveResult::success)
    {
        return GyroDriveResult::homingError;
    }

    axes[pitch]->setRpm(kPitchMaxRpm);
    moveDegree(0, -30 + kPitchZeroAdjustDegree);
    while (axes[pitch]->getStepsLeft() != 0)
    {
        runAllAxes();
    }
    stopAllAxes();

    axes[pitch]->resetPosition();

    DEBUGLOG_PRINTLN("End homing Pitch axis");
    return GyroDriveResult::success;
}

GyroDrive::GyroDriveResult GyroDrive::lookForZeroChange(Axis axis, int32_t degree, bool targetZeroedState, bool inhibitSpeedChange)
{
    if (axis == roll)
    {
        moveDegree(degree, 0, inhibitSpeedChange);
        while (rollZeroedState != targetZeroedState && axes[axis]->getStepsLeft() != 0)
        {
            runAllAxes();
        }
    }
    else
    {
        moveDegree(0, degree, inhibitSpeedChange);
        while (pitchZeroedState != targetZeroedState && axes[axis]->getStepsLeft() != 0)
        {
            runAllAxes();
        }
    }
    bool fullSwing = axes[axis]->getStepsLeft() == 0;
    stopAllAxes();
    if (fullSwing)
    { // We probably hit the hard endstop
        return GyroDriveResult::homingError;
    }
    return GyroDriveResult::success;
}