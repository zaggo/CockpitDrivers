#include "TurnCoordinator.h"
#include "DebugLog.h"
#include <EEPROM.h>

static const uint32_t kTurnCoordinatorConfigMagic = 0x54434E31; // 'T','C','N','1'
static const uint16_t kTurnCoordinatorConfigVersion = 1;
static const uint16_t kTurnCoordinatorEepromAddress = 0;

// CAN updates arrive at ~50Hz, so most moves only span a few steps and never
// leave the slow ramp-up zone of vid6608's default table (reaches top speed
// only after 800 steps). Same start delay (stall-safe) and top speed (proven)
// as the default table, but ramps to cruise within 32 steps instead of 800.
// Copied from VerticalSpeedCAN.
static vid6608::AccelTable kTurnCoordinatorAccelTable[] = {
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

TurnCoordinator::TurnCoordinator()
{
    pinMode(kRstPin, OUTPUT);
    digitalWrite(kRstPin, LOW);
    for (uint8_t i = 0; i < needleCount; i++)
    {
        motors[i] = new vid6608(kStepPins[i], kDirPins[i], kSteps);
        motors[i]->setAccelTable(kTurnCoordinatorAccelTable);
    }

    pinMode(kLightPin, OUTPUT);
    analogWrite(kLightPin, 0);

    for (int i = 0; i < 3; i++)
    {
        delay(250);
        analogWrite(kLightPin, 255);
        delay(500);
        analogWrite(kLightPin, 0);
    }

    // One /RST for both channels: release it once, then home the motors one
    // after the other. zero() is blocking and runs once at startup.
    digitalWrite(kRstPin, HIGH);
    for (uint8_t i = 0; i < needleCount; i++)
    {
        motors[i]->zero();
    }

    loadConfig();
}

TurnCoordinator::~TurnCoordinator()
{
    for (uint8_t i = 0; i < needleCount; i++)
    {
        delete motors[i];
    }
}

void TurnCoordinator::loadConfig()
{
    EEPROM.get(kTurnCoordinatorEepromAddress, config);
    if (config.magic != kTurnCoordinatorConfigMagic || config.version != kTurnCoordinatorConfigVersion)
    {
        DEBUGLOG_PRINTLN(F("TC: No valid EEPROM config, writing defaults"));
        config.magic = kTurnCoordinatorConfigMagic;
        config.version = kTurnCoordinatorConfigVersion;
        for (uint8_t i = 0; i < needleCount; i++)
        {
            calibrationWipe(config.tables[i]);
        }
        EEPROM.put(kTurnCoordinatorEepromAddress, config);
        return;
    }

    // Same magic and version but a nonsensical count in either table: refuse
    // to index past the array and fall back to factory settings for both. A
    // count of 0 is legal - a wiped table holds no anchor point.
    bool corrupt = false;
    for (uint8_t i = 0; i < needleCount; i++)
    {
        if (config.tables[i].count > kMaxCalibrationPoints)
        {
            corrupt = true;
        }
    }
    if (corrupt)
    {
        DEBUGLOG_PRINTLN(F("TC: EEPROM calibration corrupt, wiping"));
        for (uint8_t i = 0; i < needleCount; i++)
        {
            calibrationWipe(config.tables[i]);
        }
        EEPROM.put(kTurnCoordinatorEepromAddress, config);
        return;
    }

    DEBUGLOG_PRINT(F("TC: EEPROM config loaded, points turn="));
    DEBUGLOG_PRINT(config.tables[turnRate].count);
    DEBUGLOG_PRINT(F(" slip="));
    DEBUGLOG_PRINTLN(config.tables[sideSlip].count);
}

void TurnCoordinator::saveConfig()
{
    EEPROM.put(kTurnCoordinatorEepromAddress, config);
    DEBUGLOG_PRINTLN(F("TC: config saved"));
}

void TurnCoordinator::home()
{
    // Feeding the known position in keeps zero() from taking the long way
    // round and bouncing off the end stop.
    for (uint8_t i = 0; i < needleCount; i++)
    {
        motors[i]->zero(motors[i]->getPosition());
    }
}

bool TurnCoordinator::isSettled(Needle needle)
{
    return motors[needle]->isSettled();
}

bool TurnCoordinator::calibratePoint(Needle needle, int16_t deg)
{
    const uint16_t step = motors[needle]->getPosition();
    if (!calibrationSet(config.tables[needle], deg, step))
    {
        DEBUGLOG_PRINTLN(F("TC: calibration table full"));
        return false;
    }
    DEBUGLOG_PRINT(F("TC: needle "));
    DEBUGLOG_PRINT(needle);
    DEBUGLOG_PRINT(F(" calibrated "));
    DEBUGLOG_PRINT(deg);
    DEBUGLOG_PRINT(F("deg at step "));
    DEBUGLOG_PRINTLN(step);
    saveConfig();
    return true;
}

void TurnCoordinator::wipeCalibration()
{
    for (uint8_t i = 0; i < needleCount; i++)
    {
        calibrationWipe(config.tables[i]);
    }
    saveConfig();
}

void TurnCoordinator::setBrightness(uint8_t brightness)
{
    analogWrite(kLightPin, brightness);
}

void TurnCoordinator::moveNeedle(Needle needle, float deg, bool calibration)
{
    if (calibration)
    {
        motors[needle]->moveTo(static_cast<uint16_t>(deg * 12.)); // 12 steps per degree
        return;
    }

    // No isMoving() bail-out: the motor retargets on the fly (see the local
    // vid6608 fork), so feeding it fresh setpoints at 50Hz keeps the needle
    // tracking smoothly instead of stopping to accept each waypoint.
    //
    // Piecewise linear across the calibration points; values beyond the
    // lowest or highest calibrated point park on it, because the dial has no
    // marks out there.
    motors[needle]->moveTo(calibrationStepFor(config.tables[needle], deg));
}

void TurnCoordinator::setTurnCoordinator(float turnRateDeg, float slipDeg)
{
    moveNeedle(turnRate, turnRateDeg);
    moveNeedle(sideSlip, slipDeg);
}

void TurnCoordinator::loop()
{
    for (uint8_t i = 0; i < needleCount; i++)
    {
        motors[i]->loop();
    }
}
