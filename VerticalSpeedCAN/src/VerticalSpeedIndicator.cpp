#include "VerticalSpeedIndicator.h"
#include "DebugLog.h"
#include <EEPROM.h>

static const uint32_t kVerticalSpeedConfigMagic = 0x56534931; // 'V','S','I','1'
static const uint16_t kVerticalSpeedConfigVersion = 1;
static const uint16_t kVerticalSpeedEepromAddress = 0;

// CAN updates arrive at ~50Hz, so most moves only span a few steps and never
// leave the slow ramp-up zone of vid6608's default table (reaches top speed
// only after 800 steps). Same start delay (stall-safe) and top speed (proven)
// as the default table, but ramps to cruise within 32 steps instead of 800.
static vid6608::AccelTable kVerticalSpeedAccelTable[] = {
    {2, 3000},
    {4, 2400},
    {6, 1900},
    {8, 1500},
    {10, 1150},
    {12, 900},
    {15, 700},
    {18, 550},
    {21, 430},
    {24, 360},
    {27, 320},
    {32, 300},
};

VerticalSpeedIndicator::VerticalSpeedIndicator()
{
    pinMode(kRstPin, OUTPUT);
    digitalWrite(kRstPin, LOW);
    motor = new vid6608(kStepPin, kDirPin, kSteps);
    motor->setAccelTable(kVerticalSpeedAccelTable);

    pinMode(kLightPin, OUTPUT);
    analogWrite(kLightPin, 0);

    for (int i = 0; i < 3; i++)
    {
        delay(250);
        analogWrite(kLightPin, 255);
        delay(500);
        analogWrite(kLightPin, 0);
    }

    digitalWrite(kRstPin, HIGH);
    motor->zero();

    loadConfig();
}

VerticalSpeedIndicator::~VerticalSpeedIndicator()
{
    delete motor;
}

void VerticalSpeedIndicator::loadConfig()
{
    EEPROM.get(kVerticalSpeedEepromAddress, config);
    if (config.magic != kVerticalSpeedConfigMagic || config.version != kVerticalSpeedConfigVersion)
    {
        DEBUGLOG_PRINTLN(F("VSI: No valid EEPROM config, writing defaults"));
        config.magic = kVerticalSpeedConfigMagic;
        config.version = kVerticalSpeedConfigVersion;
        calibrationWipe(config.table);
        EEPROM.put(kVerticalSpeedEepromAddress, config);
    }
    else if (config.table.count > kMaxCalibrationPoints)
    {
        // Same magic and version but a nonsensical count: refuse to index past
        // the array and fall back to factory settings. A count of 0 is legal
        // here - unlike the ASI, a wiped VSI table holds no anchor point.
        DEBUGLOG_PRINTLN(F("VSI: EEPROM calibration corrupt, wiping"));
        calibrationWipe(config.table);
        EEPROM.put(kVerticalSpeedEepromAddress, config);
    }
    else
    {
        DEBUGLOG_PRINTLN(F("VSI: EEPROM config loaded"));
    }
    DEBUGLOG_PRINT(F("VSI: calibration points="));
    DEBUGLOG_PRINTLN(config.table.count);
}

void VerticalSpeedIndicator::saveConfig()
{
    EEPROM.put(kVerticalSpeedEepromAddress, config);
    DEBUGLOG_PRINT(F("VSI: config saved — calibration points="));
    DEBUGLOG_PRINTLN(config.table.count);
}

VerticalSpeedIndicator::APIResult VerticalSpeedIndicator::home()
{
    // Feeding the known position in keeps zero() from taking the long way
    // round and bouncing off the end stop.
    motor->zero(motor->getPosition());
    return success;
}

bool VerticalSpeedIndicator::calibratePoint(int16_t fpm)
{
    const uint16_t step = motor->getPosition();
    if (!calibrationSet(config.table, fpm, step))
    {
        DEBUGLOG_PRINTLN(F("VSI: calibration table full"));
        return false;
    }
    DEBUGLOG_PRINT(F("VSI: calibrated "));
    DEBUGLOG_PRINT(fpm);
    DEBUGLOG_PRINT(F("fpm at step "));
    DEBUGLOG_PRINTLN(step);
    saveConfig();
    return true;
}

VerticalSpeedIndicator::APIResult VerticalSpeedIndicator::wipeCalibration()
{
    calibrationWipe(config.table);
    saveConfig();
    return success;
}

VerticalSpeedIndicator::APIResult VerticalSpeedIndicator::setBrightness(uint8_t brightness)
{
    analogWrite(kLightPin, brightness);
    return success;
}

VerticalSpeedIndicator::APIResult VerticalSpeedIndicator::moveNeedle(float fpm, bool calibration)
{
    if (calibration)
    {
        DEBUGLOG_PRINT(F("Calibrate VSI needle to degree "));
        DEBUGLOG_PRINTLN(fpm);
        motor->moveTo(static_cast<uint16_t>(fpm * 12.)); // 12 steps per degree
        return success;
    }

    // No isMoving() bail-out: the motor retargets on the fly (see the local
    // vid6608 fork), so feeding it fresh setpoints at 50Hz keeps the needle
    // tracking smoothly instead of stopping to accept each waypoint.

    // Piecewise linear across the calibration points; rates beyond the lowest
    // or highest calibrated point park on it, because the dial has no marks
    // out there.
    const uint16_t step = calibrationStepFor(config.table, fpm);
    DEBUGLOG_PRINT(F("Move VSI needle to "));
    DEBUGLOG_PRINT(fpm);
    DEBUGLOG_PRINT(F("fpm adjusted to step "));
    DEBUGLOG_PRINTLN(step);
    motor->moveTo(step);
    return success;
}

VerticalSpeedIndicator::APIResult VerticalSpeedIndicator::loop()
{
    motor->loop();
    return success;
}

bool VerticalSpeedIndicator::isMoving()
{
    return motor->isMoving();
}
