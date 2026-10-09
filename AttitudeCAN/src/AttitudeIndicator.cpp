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

// --- homing (filled in by Task 5) -------------------------------------------

void AttitudeIndicator::homingMoveRollSteps(int32_t, RpmKeys) {}
void AttitudeIndicator::homingMoveRoll(int16_t, RpmKeys) {}
void AttitudeIndicator::homingMovePitch(int16_t, RpmKeys) {}
void AttitudeIndicator::fetchHallStates() {}
AttitudeIndicator::AttitudeResult AttitudeIndicator::lookForZeroChange(AttitudeAxis, bool) { return homingTimeout; }
AttitudeIndicator::AttitudeResult AttitudeIndicator::nextHomingState() { return homingTimeout; }
void AttitudeIndicator::runHomingStep() {}
void AttitudeIndicator::logHomingPhase() {}
void AttitudeIndicator::beginHoming() {}
