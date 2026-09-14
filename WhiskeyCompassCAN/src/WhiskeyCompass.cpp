#include "WhiskeyCompass.h"
#include <EEPROM.h>
#include "DebugLog.h"

static const uint32_t kCompassConfigMagic = 0x574B4331; // 'W','K','C','1'
static const uint16_t kCompassConfigVersion = 1;
static const uint16_t kCompassEepromAddress = 0;

static const int32_t kDegreeFullRotation = 360L;

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
}

WhiskeyCompass::WhiskeyCompass()
{
    loadConfig();

    pinMode(kHallPin, INPUT_PULLUP);

    for (uint8_t i = 0; i < kLightCount; i++)
    {
        pinMode(kLightPins[i], OUTPUT);
        analogWrite(kLightPins[i], 0);
    }

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
    card->run(micros());
}

void WhiskeyCompass::setBrightness(uint8_t brightness)
{
    for (uint8_t i = 0; i < kLightCount; i++)
    {
        analogWrite(kLightPins[i], brightness);
    }
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
    zeroedState = (digitalRead(kHallPin) == LOW);
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
            // Superseded by the capture in returnToZeroEnd below - this value is
            // unconditionally overwritten and does not feed the midpoint result.
            zeroEndPosition = normalizeStepPosition(card->getPosition(), card->getTotalSteps());
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
            // Normalise the DIFFERENCE, not the operands: this is the modular
            // distance from the window's start edge to its end edge, and halving
            // it lands on the middle. Subtracting two already-normalised
            // positions as int32_t instead would make the result negative — and
            // park the card most of a turn away — whenever the end edge is
            // captured at a lower position than the start edge.
            const uint32_t total = card->getTotalSteps();
            const int32_t windowSteps = (int32_t)normalizeStepPosition(
                (int32_t)zeroEndPosition - (int32_t)zeroStartPosition, total);
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
    homingState = unknown;
    nextHomingState();
    homingActive = true;
    DEBUGLOG_PRINTLN(F("WKC: homing started"));
}

void WhiskeyCompass::runHomingStep()
{
    const CompassResult result = nextHomingState();
    if (result != success)
    {
        DEBUGLOG_PRINTLN(F("WKC: homing failed"));
        homingState = timeout;
        homingActive = false;
        return;
    }

    if (homingState == homed)
    {
        homingActive = false;
        isHomed = true;
        DEBUGLOG_PRINTLN(F("WKC: homed"));
    }
}
