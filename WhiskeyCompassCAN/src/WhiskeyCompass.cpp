#include "WhiskeyCompass.h"
#include <EEPROM.h>
#include "DebugLog.h"

static const uint32_t kCompassConfigMagic = 0x574B4331; // 'W','K','C','1'
static const uint16_t kCompassConfigVersion = 1;
static const uint16_t kCompassEepromAddress = 0;

static const int32_t kDegreeFullRotation = 360L;

// A Hall detection window spans a few degrees of card rotation. A computed
// "window" anywhere near a full turn therefore did not come from the sensor's
// two edges at all — it comes from a phase latching before the card stepped,
// which leaves one edge captured at position 0 and turns the modular difference
// into most of a revolution. Halving that would park the card ~180 degrees off
// and still report success, so treat it as a failed homing run instead.
static const uint32_t kMaxPlausibleWindowFraction = 8; // total/8 = 45 degrees

// ...and the same argument from the other end: a window of a step or two means
// two consecutive phases were satisfied with essentially no travel between them,
// so the "midpoint" carries no information about where the window actually is.
static const int32_t kMinPlausibleWindowSteps = 4; // ~0.35 degrees

// Failed homing is signalled on the panel lights: 1s on, 1s off, forever.
static const uint32_t kFailBlinkIntervalMs = 1000;

// The gateway can legitimately dim the panel all the way down, which would hide
// the fault blink. Blink at least this bright so a failed card is always visible,
// while a brighter panel setting still wins.
static const uint8_t kFailBlinkMinBrightness = 64;

void WhiskeyCompass::applyConfigDefaults()
{
    config.magic = kCompassConfigMagic;
    config.version = kCompassConfigVersion;
    config.zeroAdjustDegree = kDefaultZeroAdjustDegree;
}

void WhiskeyCompass::loadConfig()
{
    EEPROM.get(kCompassEepromAddress, config);
    if (config.magic != kCompassConfigMagic || config.version != kCompassConfigVersion)
    {
        DEBUGLOG_PRINTLN(F("WKC: no valid EEPROM config, writing defaults"));
        applyConfigDefaults();
        EEPROM.put(kCompassEepromAddress, config);
    }
    else
    {
        DEBUGLOG_PRINTLN(F("WKC: EEPROM config loaded"));
    }
}

void WhiskeyCompass::saveConfig()
{
    EEPROM.put(kCompassEepromAddress, config);
    DEBUGLOG_PRINTLN(F("WKC: config saved"));
}

void WhiskeyCompass::wipeCalibration()
{
    applyConfigDefaults();
    saveConfig();
    // Homing folded the OLD offset into the card's origin (moveToAdjustedZero
    // moves by it, then resets the position there). Wiping the offset does not
    // move the card, so every heading would now be wrong by exactly the amount
    // just wiped - with isHomed still true and nothing saying so. Force a re-home.
    isHomed = false;
}

WhiskeyCompass::WhiskeyCompass()
{
    loadConfig();

    pinMode(kHallPin, INPUT_PULLUP);

    for (uint8_t i = 0; i < kLightCount; i++)
    {
        pinMode(kLightPins[i], OUTPUT);
    }
    applyLights(0);

    card = new CheapStepper(kStepperPins[0], kStepperPins[1],
                            kStepperPins[2], kStepperPins[3], kCardInversed);
    card->setTotalSteps(kTotalSteps);
    card->setRpm(kRpmLimits[maxRpm]);
    card->resetPosition();
}

void WhiskeyCompass::loop()
{
    if (homingActive)
    {
        runHomingStep();
    }
    updateFailBlink();
    card->run(micros());
}

void WhiskeyCompass::applyLights(uint8_t brightness)
{
    for (uint8_t i = 0; i < kLightCount; i++)
    {
        analogWrite(kLightPins[i], brightness);
    }
}

void WhiskeyCompass::setBrightness(uint8_t brightness)
{
    // Always remember the commanded level, even while the fault blink owns the
    // lights: it is what the blink's on-phase uses, and what the panel returns
    // to once homing succeeds.
    commandedBrightness = brightness;
    if (!homingFailed)
    {
        applyLights(brightness);
    }
}

void WhiskeyCompass::updateFailBlink()
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
    applyLights(failBlinkOn ? max(commandedBrightness, kFailBlinkMinBrightness) : 0);
}

void WhiskeyCompass::stop()
{
    card->stop();
}

void WhiskeyCompass::off()
{
    card->off();
}

void WhiskeyCompass::moveSteps(int32_t steps)
{
    card->newMove(steps > 0, (uint32_t)labs(steps));
}

void WhiskeyCompass::moveDegree(int32_t degree)
{
    moveSteps(degree * (int32_t)card->getTotalSteps() / kDegreeFullRotation);
}

WhiskeyCompass::CompassResult WhiskeyCompass::moveToHeading(double degMag)
{
    if (!isHomed)
    {
        return notHomed;
    }

    // The stored zero offset is NOT added here. Homing's moveToAdjustedZero phase
    // already drove the card onto the painted N mark and reset the position there,
    // so step position 0 is the painted zero. Adding the offset again would
    // double-apply it and leave the card off by that much.
    const uint32_t total = card->getTotalSteps();
    const uint32_t target = degreesToSteps(degMag, total);
    const uint32_t current = normalizeStepPosition(card->getPosition(), total);

    // A new setpoint replaces whatever move is in flight, recomputed from the
    // current position. At 50Hz a queue would only pile up stale headings.
    moveSteps(shortestPathSteps(current, target, total));
    return success;
}

bool WhiskeyCompass::calibrateZero()
{
    if (!isHomed)
    {
        DEBUGLOG_PRINTLN(F("WKC: not homed, cannot store zero"));
        return false;
    }

    // Freeze the position before reading it: with a move still in flight the
    // card would keep travelling past the point we are about to declare north,
    // making the stored offset wrong by whatever travel remained.
    card->stop();

    // getPosition() is int32_t and may be negative; normalising first keeps the
    // fold in jogDegreesFromPosition well-defined.
    const uint32_t total = card->getTotalSteps();
    const int32_t jog = jogDegreesFromPosition(normalizeStepPosition(card->getPosition(), total),
                                               total);
    config.zeroAdjustDegree = accumulateZeroAdjust(config.zeroAdjustDegree, jog);
    saveConfig();

    // The card is now standing on what we just declared to be north.
    card->resetPosition();
    return true;
}

void WhiskeyCompass::fetchZeroedState()
{
    const bool state = (digitalRead(kHallPin) == LOW);
    // Log edges only. The card steps every 1.4ms at the slow homing rate, so a
    // print per poll would both flood the line and stall the stepper; a print
    // per edge is a handful of lines per run and shows exactly where in the
    // phase's own step frame the sensor changed.
    if (state != zeroedState)
    {
        DEBUGLOG_PRINT(F("WKC: hall "));
        DEBUGLOG_PRINT(state ? F("ON") : F("OFF"));
        DEBUGLOG_PRINT(F(" @ "));
        DEBUGLOG_PRINTLN(card->getPosition());
    }
    zeroedState = state;
}

void WhiskeyCompass::logHomingPhase()
{
#if DEBUGLOG_ENABLE
    DEBUGLOG_PRINT(F("WKC: phase "));
    switch (homingState)
    {
    case unknown:
        DEBUGLOG_PRINT(F("unknown"));
        break;
    case leaveZero:
        DEBUGLOG_PRINT(F("leaveZero"));
        break;
    case searchZero:
        DEBUGLOG_PRINT(F("searchZero"));
        break;
    case searchZeroEnd:
        DEBUGLOG_PRINT(F("searchZeroEnd"));
        break;
    case returnToZeroEnd:
        DEBUGLOG_PRINT(F("returnToZeroEnd"));
        break;
    case searchZeroStart:
        DEBUGLOG_PRINT(F("searchZeroStart"));
        break;
    case moveToTrueZero:
        DEBUGLOG_PRINT(F("moveToTrueZero"));
        break;
    case moveToAdjustedZero:
        DEBUGLOG_PRINT(F("moveToAdjustedZero"));
        break;
    case homed:
        DEBUGLOG_PRINT(F("homed"));
        break;
    case timeout:
        DEBUGLOG_PRINT(F("timeout"));
        break;
    }
    // Travel budget the phase was just handed. A phase that reaches its target
    // state with this still near its start value never moved - that is the
    // zero-travel capture that produces a bogus zero window.
    DEBUGLOG_PRINT(F(" left "));
    DEBUGLOG_PRINT(card->getStepsLeft());
    DEBUGLOG_PRINT(F(" hall "));
    DEBUGLOG_PRINTLN(zeroedState ? F("ON") : F("OFF"));
#endif
}

WhiskeyCompass::CompassResult WhiskeyCompass::lookForZeroChange(bool targetZeroedState)
{
    fetchZeroedState();
    if (zeroedState != targetZeroedState && card->getStepsLeft() != 0)
    {
        return success; // still searching
    }
    if (zeroedState != targetZeroedState)
    {
        DEBUGLOG_PRINTLN(F("WKC: homing timeout"));
        return homingTimeout; // ran out of commanded travel
    }
    return cardStateReached;
}

WhiskeyCompass::CompassResult WhiskeyCompass::nextHomingState()
{
    CompassResult zeroState;
    switch (homingState)
    {
    case unknown:
        fetchZeroedState();
        card->stop();
        card->resetPosition();
        card->setRpm(kRpmLimits[maxRpm]);
        // 370 degrees is deliberately more than a full turn, so the sensor is
        // guaranteed to be passed no matter where the card started.
        moveDegree(370);
        homingState = leaveZero;
        return success;

    case leaveZero:
        zeroState = lookForZeroChange(false);
        if (zeroState == cardStateReached)
        {
            card->stop();
            card->resetPosition();
            moveDegree(370);
            homingState = searchZero;
            return success;
        }
        return zeroState;

    case searchZero:
        zeroState = lookForZeroChange(true);
        if (zeroState == cardStateReached)
        {
            card->stop();
            card->resetPosition();
            card->setRpm(kRpmLimits[minRpm]);
            moveDegree(60);
            homingState = searchZeroEnd;
            return success;
        }
        return zeroState;

    case searchZeroEnd:
        zeroState = lookForZeroChange(false);
        if (zeroState == cardStateReached)
        {
            card->stop();
            card->resetPosition();
            moveDegree(-65);
            homingState = returnToZeroEnd;
            return success;
        }
        return zeroState;

    case returnToZeroEnd:
        zeroState = lookForZeroChange(true);
        if (zeroState == cardStateReached)
        {
            zeroEndPosition = normalizeStepPosition(card->getPosition(), card->getTotalSteps());
            card->stop();
            card->resetPosition();
            moveDegree(-65);
            homingState = searchZeroStart;
            return success;
        }
        return zeroState;

    case searchZeroStart:
        zeroState = lookForZeroChange(false);
        if (zeroState == cardStateReached)
        {
            const uint32_t zeroStartPosition =
                normalizeStepPosition(card->getPosition(), card->getTotalSteps());
            card->stop();
            card->resetPosition();
            card->setRpm(kRpmLimits[maxRpm]);
            // True zero is the middle of the Hall window. Taking the first edge
            // instead would make homing depend on approach direction.
            //
            // returnToZeroEnd reset the position ON the window's end edge, so in
            // the frame zeroStartPosition was captured in the end edge sits at 0
            // and the start edge at a negative offset. The modular distance from
            // start to end is therefore -zeroStartPosition, and halving it lands
            // on the middle. Do NOT reintroduce zeroEndPosition here: it is
            // captured BEFORE that reset, in the previous frame, so the
            // difference would silently be short by however many steps the
            // reverse sweep took to re-assert the sensor (backlash + hysteresis).
            const uint32_t total = card->getTotalSteps();
            const int32_t windowSteps =
                (int32_t)normalizeStepPosition(-(int32_t)zeroStartPosition, total);
            // Raw captures too, not just the halved result: an edge captured at
            // 0 (or at total-1) is the signature of a phase that latched before
            // the card stepped, and the halved value alone cannot show that.
            // "rev" is how far returnToZeroEnd travelled before the sensor came
            // back - backlash plus hysteresis, and a sanity check on its own.
            DEBUGLOG_PRINT(F("WKC: window rev "));
            DEBUGLOG_PRINT(zeroEndPosition);
            DEBUGLOG_PRINT(F(" start "));
            DEBUGLOG_PRINT(zeroStartPosition);
            DEBUGLOG_PRINT(F(" span "));
            DEBUGLOG_PRINTLN(windowSteps);
            // Bounded on BOTH sides. Too wide is the phase-latched-at-0 case
            // described at kMaxPlausibleWindowFraction. Too narrow is the same
            // class of artifact from the other end - a chattering edge that
            // satisfied two consecutive phases with no travel between them - and
            // it carries no information about where the window's middle is.
            if ((uint32_t)windowSteps > total / kMaxPlausibleWindowFraction ||
                windowSteps < kMinPlausibleWindowSteps)
            {
                // A window this wide - or this narrow - cannot be the Hall
                // sensor's real detection window (see the two k*PlausibleWindow*
                // constants above); most likely a phase latched before the card
                // had stepped. The card and stepper are already stopped and reset
                // above, so just fail homing instead of parking off north.
                DEBUGLOG_PRINTLN(F("WKC: implausible zero window, homing failed"));
                return homingTimeout;
            }
            const int32_t zeroAdjust = windowSteps / 2L;
            DEBUGLOG_PRINT(F("WKC: zero adjust steps "));
            DEBUGLOG_PRINTLN(zeroAdjust);
            moveSteps(zeroAdjust);
            homingState = moveToTrueZero;
            return success;
        }
        return zeroState;

    case moveToTrueZero:
        if (card->getStepsLeft() != 0)
        {
            return success;
        }
        card->resetPosition();
        moveDegree(config.zeroAdjustDegree);
        homingState = moveToAdjustedZero;
        return success;

    case moveToAdjustedZero:
        if (card->getStepsLeft() != 0)
        {
            return success;
        }
        card->resetPosition();
        homingState = homed;
        return success;

    case homed:
        return success;

    case timeout:
        return homingTimeout;
    }
    return homingTimeout;
}

void WhiskeyCompass::beginHoming()
{
    if (homingActive)
    {
        return;
    }

    card->stop();
    isHomed = false;
    // A retry clears the fault blink and hands the lights back to the panel
    // brightness; a second failure switches it on again.
    homingFailed = false;
    failBlinkOn = false;
    applyLights(commandedBrightness);
    homingState = unknown;
    DEBUGLOG_PRINTLN(F("WKC: homing started"));
    nextHomingState();
    logHomingPhase();
    homingActive = true;
}

void WhiskeyCompass::runHomingStep()
{
    // Phase transitions are the interesting events, so log on change rather
    // than once per loop iteration - this runs at the full loop rate.
    const HomingPhase phaseBefore = homingState;
    const CompassResult result = nextHomingState();
    if (homingState != phaseBefore)
    {
        logHomingPhase();
    }
    if (result != success)
    {
        // A timeout leaves homingState on the phase that ran out of travel, so
        // log it before it is overwritten - that name is the whole diagnosis.
        logHomingPhase();
        DEBUGLOG_PRINTLN(F("WKC: homing failed"));
        homingState = timeout;
        homingActive = false;
        // Blink the panel lights until someone retries homing: the card is
        // parked at an unknown heading and will refuse setpoints, which is
        // otherwise invisible on the instrument itself.
        homingFailed = true;
        failBlinkOn = true;
        failBlinkLastToggleMs = millis();
        applyLights(max(commandedBrightness, kFailBlinkMinBrightness));
        return;
    }

    if (homingState == homed)
    {
        homingActive = false;
        isHomed = true;
        DEBUGLOG_PRINTLN(F("WKC: homed"));
    }
}
