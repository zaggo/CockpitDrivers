#include "AttitudeIndicator.h"
#include <EEPROM.h>
#include "DebugLog.h"

static const uint32_t kAttitudeConfigMagic = 0x41544331; // 'A','T','C','1'
static const uint16_t kAttitudeConfigVersion = 1;
static const uint16_t kAttitudeEepromAddress = 0;

// Failed homing is signalled on the panel light: 1s on, 1s off, forever.
static const uint32_t kFailBlinkIntervalMs = 1000;
// The gateway may legitimately dim the panel to zero, which would hide the
// fault blink. Blink at least this bright; a brighter panel setting still wins.
static const uint8_t kFailBlinkMinBrightness = 64;

// --- config -----------------------------------------------------------------

void AttitudeIndicator::applyConfigDefaults()
{
    config.magic = kAttitudeConfigMagic;
    config.version = kAttitudeConfigVersion;
    for (uint8_t i = 0; i < attitudeAxisCount; i++)
    {
        config.zeroAdjustDegree[i] = kDefaultZeroAdjustDegree[i];
    }
}

void AttitudeIndicator::loadConfig()
{
    EEPROM.get(kAttitudeEepromAddress, config);
    if (config.magic != kAttitudeConfigMagic || config.version != kAttitudeConfigVersion)
    {
        DEBUGLOG_PRINTLN(F("ATT: no valid EEPROM config, writing defaults"));
        applyConfigDefaults();
        EEPROM.put(kAttitudeEepromAddress, config);
    }
    else
    {
        DEBUGLOG_PRINTLN(F("ATT: EEPROM config loaded"));
    }
}

void AttitudeIndicator::saveConfig()
{
    EEPROM.put(kAttitudeEepromAddress, config);
    DEBUGLOG_PRINTLN(F("ATT: config saved"));
}

void AttitudeIndicator::wipeCalibration()
{
    applyConfigDefaults();
    saveConfig();
    // Homing folded the OLD offsets into both origins. Wiping them does not
    // move anything, so every setpoint would now be off by exactly the wiped
    // amount with isHomed still true. Force a re-home.
    isHomed = false;
}

// --- construction / loop ----------------------------------------------------

AttitudeIndicator::AttitudeIndicator()
{
    loadConfig();

    pinMode(kLightPin, OUTPUT);
    applyLight(0);

    for (uint8_t i = 0; i < attitudeAxisCount; i++)
    {
        pinMode(kHallPins[i], INPUT_PULLUP);
        axes[i] = new CheapStepper(kStepperPins[i][0], kStepperPins[i][1],
                                   kStepperPins[i][2], kStepperPins[i][3],
                                   kStepperInversed[i]);
        axes[i]->setTotalSteps(kTotalSteps);
        // setRpm returns the step delay it computed; remember both rates so a
        // move can pick them without recomputing.
        stepDelay[i][minRpm] = axes[i]->setRpm(kRpmLimits[i][minRpm]);
        stepDelay[i][maxRpm] = axes[i]->setRpm(kRpmLimits[i][maxRpm]);
        axes[i]->resetPosition();
    }
    fetchHallStates();
}

void AttitudeIndicator::loop()
{
    if (homingActive)
    {
        runHomingStep();
    }
    updateFailBlink();
    const uint32_t now = micros();
    axes[roll]->run(now);
    axes[pitch]->run(now);
}

// --- light ------------------------------------------------------------------

void AttitudeIndicator::applyLight(uint8_t brightness)
{
    analogWrite(kLightPin, brightness);
}

void AttitudeIndicator::setBrightness(uint8_t brightness)
{
    // Always remember the commanded level, even while the fault blink owns the
    // light: it is what the blink's on-phase uses, and what the panel returns
    // to once homing succeeds.
    commandedBrightness = brightness;
    if (!homingFailed)
    {
        applyLight(brightness);
    }
}

void AttitudeIndicator::updateFailBlink()
{
    if (!homingFailed)
    {
        return;
    }
    const uint32_t now = millis();
    if (now - failBlinkLastToggleMs < kFailBlinkIntervalMs)
    {
        return;
    }
    failBlinkLastToggleMs = now;
    failBlinkOn = !failBlinkOn;
    applyLight(failBlinkOn ? max(commandedBrightness, kFailBlinkMinBrightness) : 0);
}

// --- motion -----------------------------------------------------------------

void AttitudeIndicator::stop()
{
    stopAll();
}

void AttitudeIndicator::off()
{
    axes[roll]->off();
    axes[pitch]->off();
}

void AttitudeIndicator::stopAll()
{
    axes[roll]->stop();
    axes[pitch]->stop();
}

void AttitudeIndicator::resetAll()
{
    axes[roll]->resetPosition();
    axes[pitch]->resetPosition();
}

void AttitudeIndicator::moveMotors(int32_t stepsRoll, int32_t stepsPitch)
{
    uint32_t delayRoll = 0;
    uint32_t delayPitch = 0;
    coordinatedDelays(stepsRoll, stepsPitch,
                      stepDelay[roll][maxRpm], stepDelay[pitch][maxRpm],
                      &delayRoll, &delayPitch);
    axes[roll]->setDelay(delayRoll);
    axes[pitch]->setDelay(delayPitch);

    // Same timestamp for both so the paced delays actually line up.
    const uint32_t now = micros();
    axes[roll]->newMove(stepsRoll > 0, (uint32_t)labs(stepsRoll), now);
    axes[pitch]->newMove(stepsPitch > 0, (uint32_t)labs(stepsPitch), now);
}

void AttitudeIndicator::moveToTargets(uint32_t rollTarget, uint32_t pitchMotorTargetPos)
{
    const uint32_t rollCurrent = normalizeStepPosition(axes[roll]->getPosition(), kTotalSteps);
    const uint32_t pitchCurrent = normalizeStepPosition(axes[pitch]->getPosition(), kTotalSteps);
    // A new setpoint replaces whatever move is in flight, recomputed from the
    // current positions. At 50Hz a queue would only pile up stale attitudes.
    moveMotors(shortestPathSteps(rollCurrent, rollTarget, kTotalSteps),
               shortestPathSteps(pitchCurrent, pitchMotorTargetPos, kTotalSteps));
}

AttitudeIndicator::AttitudeResult AttitudeIndicator::moveToAttitude(double pitchDeg, double rollDeg)
{
    if (!isHomed)
    {
        return notHomed;
    }
    commandedPitchDeg = pitchDeg;
    commandedRollDeg = rollDeg;

    // The stored zero offsets are NOT added here: homing already drove each axis
    // onto its adjusted zero and reset the position there, so position 0 is
    // level / wings level. Adding them again would double-apply them.
    const uint32_t rollTarget = normalizeStepPosition(
        signedDegreesToSteps(rollDisplayDegree(rollDeg, kRollInversed), kTotalSteps), kTotalSteps);
    const int32_t pitchSteps =
        signedDegreesToSteps(pitchDisplayDegree(pitchDeg, kPitchMaxDegree, kPitchScale), kTotalSteps);
    moveToTargets(rollTarget, pitchMotorTarget(pitchSteps, rollTarget, kTotalSteps));
    return success;
}

AttitudeIndicator::AttitudeResult AttitudeIndicator::moveAxisToDegree(AttitudeAxis axis, double deg)
{
    if (axis == roll)
    {
        return moveToAttitude(commandedPitchDeg, deg);
    }
    return moveToAttitude(deg, commandedRollDeg);
}

double AttitudeIndicator::displayRollDegree() const
{
    const int32_t steps = jogDegreesFromPosition(
        normalizeStepPosition(axes[roll]->getPosition(), kTotalSteps), kTotalSteps);
    return rollDisplayDegree((double)steps, kRollInversed);
}

double AttitudeIndicator::displayPitchDegree() const
{
    const int32_t steps = pitchDisplaySteps(axes[pitch]->getPosition(),
                                            axes[roll]->getPosition(), kTotalSteps);
    return ((double)steps * 360.0 / (double)kTotalSteps) / kPitchScale;
}

// --- calibration ------------------------------------------------------------

bool AttitudeIndicator::calibrateZero(AttitudeAxis axis)
{
    if (!isHomed)
    {
        DEBUGLOG_PRINTLN(F("ATT: not homed, cannot store zero"));
        return false;
    }
    // Freeze before reading: a move still in flight would carry the axis past
    // the point we are about to declare zero.
    stopAll();

    const int32_t rollPos = axes[roll]->getPosition();
    const int32_t pitchPos = axes[pitch]->getPosition();
    int32_t jog;
    if (axis == roll)
    {
        jog = jogDegreesFromPosition(normalizeStepPosition(rollPos, kTotalSteps), kTotalSteps);
    }
    else
    {
        // The pitch offset is a DISPLAY quantity: motor + roll, not the motor alone.
        jog = jogDegreesFromPosition(normalizeStepPosition(pitchPos + rollPos, kTotalSteps), kTotalSteps);
    }
    config.zeroAdjustDegree[axis] = accumulateZeroAdjust(config.zeroAdjustDegree[axis], jog);
    saveConfig();

    // Re-base the positions on what we just declared zero, keeping display =
    // motor + roll intact: declaring roll zero shifts the pitch motor's frame by
    // the old roll, declaring pitch zero moves the pitch motor to -roll.
    if (axis == roll)
    {
        axes[roll]->resetPosition();
        axes[pitch]->resetPosition(normalizeStepPosition(pitchPos + rollPos, kTotalSteps));
    }
    else
    {
        axes[pitch]->resetPosition(normalizeStepPosition(-rollPos, kTotalSteps));
    }
    return true;
}

// --- homing -----------------------------------------------------------------
//
// ORDER IS A MECHANICAL REQUIREMENT. Roll is homed first, and every roll move
// drags the pitch motor the other way by the same amount, so the pitch display
// does not move while roll searches its sensor. The pitch display has stops;
// searching roll with the pitch motor standing still would turn the display
// along with the frame and drive it into one. Pitch is homed only once roll
// stands at 0, at which point pitch motor == pitch display and the pitch
// phases need no correction. This is GyroDrive::homeAllAxis() made non-blocking.

// A window of a step or two means two consecutive phases were satisfied with
// essentially no travel between them (a chattering edge); its midpoint carries
// no information. The upper bound lives in homingWindowCentre().
static const int32_t kMinPlausibleWindowSteps = 4; // ~0.35 degrees

void AttitudeIndicator::homingMoveRollSteps(int32_t steps, RpmKeys rate)
{
    // Same delay on both motors: they must stay in lockstep so a stop() in the
    // middle of a phase leaves the pitch display exactly where it was.
    axes[roll]->setDelay(stepDelay[roll][rate]);
    axes[pitch]->setDelay(stepDelay[roll][rate]);
    const uint32_t now = micros();
    axes[roll]->newMove(steps > 0, (uint32_t)labs(steps), now);
    axes[pitch]->newMove(steps < 0, (uint32_t)labs(steps), now);
}

void AttitudeIndicator::homingMoveRoll(int16_t degree, RpmKeys rate)
{
    homingMoveRollSteps(signedDegreesToSteps((double)degree, kTotalSteps), rate);
}

void AttitudeIndicator::homingMovePitch(int16_t degree, RpmKeys rate)
{
    const int32_t steps = signedDegreesToSteps((double)degree, kTotalSteps);
    axes[pitch]->setDelay(stepDelay[pitch][rate]);
    axes[pitch]->newMove(steps > 0, (uint32_t)labs(steps), micros());
}

void AttitudeIndicator::fetchHallStates()
{
    for (uint8_t i = 0; i < attitudeAxisCount; i++)
    {
        const bool state = (digitalRead(kHallPins[i]) == LOW);
        // Log edges only: a print per poll would flood the line and stall the
        // steppers; a print per edge is a handful of lines per run.
        if (state != hallState[i])
        {
            DEBUGLOG_PRINT(i == roll ? F("ATT: hall roll ") : F("ATT: hall pitch "));
            DEBUGLOG_PRINT(state ? F("ON") : F("OFF"));
            DEBUGLOG_PRINT(F(" @ "));
            DEBUGLOG_PRINTLN(axes[i]->getPosition());
        }
        hallState[i] = state;
    }
}

AttitudeIndicator::AttitudeResult AttitudeIndicator::lookForZeroChange(AttitudeAxis axis, bool targetState)
{
    fetchHallStates();
    if (hallState[axis] != targetState && axes[axis]->getStepsLeft() != 0)
    {
        return success; // still searching
    }
    if (hallState[axis] != targetState)
    {
        // Ran out of commanded travel: a stop, or a sensor that never changes.
        DEBUGLOG_PRINTLN(F("ATT: homing timeout"));
        return homingTimeout;
    }
    return axisStateReached;
}

AttitudeIndicator::AttitudeResult AttitudeIndicator::nextHomingState()
{
    AttitudeResult state;
    switch (homingState)
    {
    // ---------------- roll ----------------
    case unknown:
        fetchHallStates();
        stopAll();
        resetAll();
        homingMoveRoll(kRollHomingLeaveDegree, maxRpm);
        homingState = rollLeaveZero;
        return success;

    case rollLeaveZero:
        state = lookForZeroChange(roll, false);
        if (state == axisStateReached)
        {
            stopAll();
            resetAll();
            homingMoveRoll(kRollHomingSearchDegree, maxRpm);
            homingState = rollSearchZero;
            return success;
        }
        return state;

    case rollSearchZero:
        state = lookForZeroChange(roll, true);
        if (state == axisStateReached)
        {
            stopAll();
            resetAll();
            homingMoveRoll(kRollHomingSlowEndDegree, minRpm);
            homingState = rollSearchZeroEnd;
            return success;
        }
        return state;

    case rollSearchZeroEnd:
        state = lookForZeroChange(roll, false);
        if (state == axisStateReached)
        {
            stopAll();
            resetAll();
            homingMoveRoll(kRollHomingSlowReturnDegree, minRpm);
            homingState = rollReturnToZeroEnd;
            return success;
        }
        return state;

    case rollReturnToZeroEnd:
        state = lookForZeroChange(roll, true);
        if (state == axisStateReached)
        {
            // Position is reset ON the window's end edge here, so the start
            // edge is captured in the next phase as a negative offset from it.
            stopAll();
            resetAll();
            homingMoveRoll(kRollHomingSlowStartDegree, minRpm);
            homingState = rollSearchZeroStart;
            return success;
        }
        return state;

    case rollSearchZeroStart:
        state = lookForZeroChange(roll, false);
        if (state == axisStateReached)
        {
            const uint32_t zeroStart = normalizeStepPosition(axes[roll]->getPosition(), kTotalSteps);
            stopAll();
            resetAll();
            // End edge sits at 0 in this frame (see above), so the window is the
            // modular distance from zeroStart forward to 0, and its centre is
            // half of that. Bounded on both sides: see homingWindowCentre() and
            // kMinPlausibleWindowSteps.
            int32_t centre = 0;
            const bool plausible = homingWindowCentre(0, zeroStart, kTotalSteps, &centre)
                                   && centre * 2 >= kMinPlausibleWindowSteps;
            DEBUGLOG_PRINT(F("ATT: roll window start "));
            DEBUGLOG_PRINT(zeroStart);
            DEBUGLOG_PRINT(F(" centre "));
            DEBUGLOG_PRINTLN(centre);
            if (!plausible)
            {
                DEBUGLOG_PRINTLN(F("ATT: implausible roll zero window, homing failed"));
                return homingTimeout;
            }
            homingMoveRollSteps(centre, maxRpm);
            homingState = rollMoveToTrueZero;
            return success;
        }
        return state;

    case rollMoveToTrueZero:
        if (axes[roll]->getStepsLeft() != 0)
        {
            return success;
        }
        resetAll();
        homingMoveRoll(config.zeroAdjustDegree[roll], maxRpm);
        homingState = rollMoveToAdjustedZero;
        return success;

    case rollMoveToAdjustedZero:
        if (axes[roll]->getStepsLeft() != 0)
        {
            return success;
        }
        // Roll now stands at 0. From here on pitch motor == pitch display.
        resetAll();
        homingMovePitch(kPitchHomingLeaveDegree, maxRpm);
        homingState = pitchLeaveZero;
        return success;

    // ---------------- pitch ----------------
    case pitchLeaveZero:
        state = lookForZeroChange(pitch, false);
        if (state == axisStateReached)
        {
            axes[pitch]->stop();
            axes[pitch]->resetPosition();
            homingMovePitch(kPitchHomingSearchDegree, maxRpm);
            homingState = pitchSearchZero;
            return success;
        }
        return state;

    case pitchSearchZero:
        state = lookForZeroChange(pitch, true);
        if (state == axisStateReached)
        {
            axes[pitch]->stop();
            axes[pitch]->resetPosition();
            homingMovePitch(kPitchHomingBackOffDegree, maxRpm);
            homingState = pitchBackOff;
            return success;
        }
        return state;

    case pitchBackOff:
        if (axes[pitch]->getStepsLeft() != 0)
        {
            return success;
        }
        axes[pitch]->resetPosition();
        homingMovePitch(kPitchHomingSlowSearchDegree, minRpm);
        homingState = pitchSlowSearchZero;
        return success;

    case pitchSlowSearchZero:
        state = lookForZeroChange(pitch, true);
        if (state == axisStateReached)
        {
            axes[pitch]->stop();
            axes[pitch]->resetPosition();
            homingMovePitch(kPitchHomingParkDegree + config.zeroAdjustDegree[pitch], maxRpm);
            homingState = pitchMoveToAdjustedZero;
            return success;
        }
        return state;

    case pitchMoveToAdjustedZero:
        if (axes[pitch]->getStepsLeft() != 0)
        {
            return success;
        }
        resetAll();
        homingState = homed;
        return success;

    case homed:
        return success;

    case timeout:
        return homingTimeout;
    }
    return homingTimeout;
}

void AttitudeIndicator::beginHoming()
{
    if (homingActive)
    {
        return;
    }
    stopAll();
    isHomed = false;
    // A retry clears the fault blink and hands the light back to the panel
    // brightness; a second failure switches it on again.
    homingFailed = false;
    failBlinkOn = false;
    applyLight(commandedBrightness);
    commandedPitchDeg = 0.0;
    commandedRollDeg = 0.0;
    homingState = unknown;
    DEBUGLOG_PRINTLN(F("ATT: homing started"));
    nextHomingState();
    logHomingPhase();
    homingActive = true;
}

void AttitudeIndicator::runHomingStep()
{
    // Phase transitions are the interesting events; log on change, not per loop.
    const HomingPhase phaseBefore = homingState;
    const AttitudeResult result = nextHomingState();
    if (homingState != phaseBefore)
    {
        logHomingPhase();
    }
    if (result != success)
    {
        // A timeout leaves homingState on the phase that ran out of travel; log
        // it before it is overwritten - that name is the whole diagnosis.
        logHomingPhase();
        DEBUGLOG_PRINTLN(F("ATT: homing failed"));
        stopAll();
        homingState = timeout;
        homingActive = false;
        homingFailed = true;
        failBlinkOn = true;
        failBlinkLastToggleMs = millis();
        applyLight(max(commandedBrightness, kFailBlinkMinBrightness));
        return;
    }
    if (homingState == homed)
    {
        homingActive = false;
        isHomed = true;
        DEBUGLOG_PRINTLN(F("ATT: homed"));
    }
}

void AttitudeIndicator::logHomingPhase()
{
#if DEBUGLOG_ENABLE
    static const char *const kPhaseNames[] = {
        "unknown", "rollLeaveZero", "rollSearchZero", "rollSearchZeroEnd",
        "rollReturnToZeroEnd", "rollSearchZeroStart", "rollMoveToTrueZero",
        "rollMoveToAdjustedZero", "pitchLeaveZero", "pitchSearchZero", "pitchBackOff",
        "pitchSlowSearchZero", "pitchMoveToAdjustedZero", "homed", "timeout"};
    DEBUGLOG_PRINT(F("ATT: phase "));
    DEBUGLOG_PRINT(kPhaseNames[homingState]);
    DEBUGLOG_PRINT(F(" left r/p "));
    DEBUGLOG_PRINT(axes[roll]->getStepsLeft());
    DEBUGLOG_PRINT(F("/"));
    DEBUGLOG_PRINT(axes[pitch]->getStepsLeft());
    DEBUGLOG_PRINT(F(" hall r/p "));
    DEBUGLOG_PRINT(hallState[roll] ? F("ON") : F("OFF"));
    DEBUGLOG_PRINT(F("/"));
    DEBUGLOG_PRINTLN(hallState[pitch] ? F("ON") : F("OFF"));
#endif
}
