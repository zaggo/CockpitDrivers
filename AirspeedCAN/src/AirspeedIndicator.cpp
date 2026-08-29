#include "AirspeedIndicator.h"
#include "DebugLog.h"
#include <EEPROM.h>

static const uint32_t kAirspeedConfigMagic = 0x41534931; // 'A','S','I','1'
// v2 replaced the single minStep/maxStep pair with the interpolation table, so
// a v1 EEPROM image fails this check and is reset to a wiped table.
static const uint16_t kAirspeedConfigVersion = 2;
static const uint16_t kAirspeedEepromAddress = 0;

// CAN updates arrive at ~50Hz, so most moves only span a few steps and never
// leave the slow ramp-up zone of vid6608's default table (reaches top speed
// only after 800 steps). Same start delay (stall-safe) and top speed (proven)
// as the default table, but ramps to cruise within 32 steps instead of 800.
static vid6608::AccelTable kAirspeedAccelTable[] = {
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

AirspeedIndicator::AirspeedIndicator()
{
    pinMode(kRstPin, OUTPUT);
    digitalWrite(kRstPin, LOW);
    motor = new vid6608(kStepPin, kDirPin, kSteps);
    motor->setAccelTable(kAirspeedAccelTable);

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

AirspeedIndicator::~AirspeedIndicator()
{
    delete motor;
}

void AirspeedIndicator::loadConfig()
{
    EEPROM.get(kAirspeedEepromAddress, config);
    if (config.magic != kAirspeedConfigMagic || config.version != kAirspeedConfigVersion)
    {
        DEBUGLOG_PRINTLN(F("ASI: No valid EEPROM config, writing defaults"));
        config.magic = kAirspeedConfigMagic;
        config.version = kAirspeedConfigVersion;
        calibrationWipe(config.table);
        EEPROM.put(kAirspeedEepromAddress, config);
    }
    else if (config.table.count == 0 || config.table.count > kMaxCalibrationPoints)
    {
        // Same magic and version but a nonsensical count: refuse to index past
        // the array and fall back to factory settings.
        DEBUGLOG_PRINTLN(F("ASI: EEPROM calibration corrupt, wiping"));
        calibrationWipe(config.table);
        EEPROM.put(kAirspeedEepromAddress, config);
    }
    else
    {
        DEBUGLOG_PRINTLN(F("ASI: EEPROM config loaded"));
    }
    DEBUGLOG_PRINT(F("ASI: calibration points="));
    DEBUGLOG_PRINTLN(config.table.count);
}

void AirspeedIndicator::saveConfig()
{
    EEPROM.put(kAirspeedEepromAddress, config);
    DEBUGLOG_PRINT(F("ASI: config saved — calibration points="));
    DEBUGLOG_PRINTLN(config.table.count);
}

AirspeedIndicator::APIResult AirspeedIndicator::home()
{
    // Feeding the known position in keeps zero() from taking the long way
    // round and bouncing off the end stop.
    motor->zero(motor->getPosition());
    return success;
}

bool AirspeedIndicator::calibratePoint(uint16_t knots)
{
    const uint16_t step = motor->getPosition();
    if (!calibrationSet(config.table, knots, step))
    {
        DEBUGLOG_PRINTLN(F("ASI: calibration table full"));
        return false;
    }
    DEBUGLOG_PRINT(F("ASI: calibrated "));
    DEBUGLOG_PRINT(knots);
    DEBUGLOG_PRINT(F("kt at step "));
    DEBUGLOG_PRINTLN(step);
    saveConfig();
    return true;
}

AirspeedIndicator::APIResult AirspeedIndicator::wipeCalibration()
{
    calibrationWipe(config.table);
    saveConfig();
    return success;
}

AirspeedIndicator::APIResult AirspeedIndicator::setBrightness(uint8_t brightness)
{
    analogWrite(kLightPin, brightness);
    return success;
}

AirspeedIndicator::APIResult AirspeedIndicator::moveNeedle(float ias, bool calibration)
{
    if (calibration)
    {
        DEBUGLOG_PRINT(F("Calibrate ASI needle to degree "));
        DEBUGLOG_PRINTLN(ias);
        motor->moveTo(static_cast<uint16_t>(ias * 12.)); // 12 steps per degree
        return success;
    }

    // No isMoving() bail-out: the motor retargets on the fly (see the local
    // vid6608 fork), so feeding it fresh setpoints at 50Hz keeps the needle
    // tracking smoothly instead of stopping to accept each waypoint.

    // Piecewise linear across the calibration points; speeds past the highest
    // calibrated point park on it, because the dial has no marks beyond.
    const uint16_t step = calibrationStepFor(config.table, ias);
    DEBUGLOG_PRINT(F("Move ASI needle to "));
    DEBUGLOG_PRINT(ias);
    DEBUGLOG_PRINT(F("kt adjusted to step "));
    DEBUGLOG_PRINTLN(step);
    motor->moveTo(step);
    return success;
}

AirspeedIndicator::APIResult AirspeedIndicator::loop()
{
    motor->loop();
    return success;
}

bool AirspeedIndicator::isMoving()
{
    return motor->isMoving();
}
