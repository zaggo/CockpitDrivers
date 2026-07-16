#include "RPMGauge.h"
#include "DebugLog.h"
#include <EEPROM.h>

static const uint32_t kRPMGaugeConfigMagic = 0x52474331; // 'R','G','C','1'
static const uint16_t kRPMGaugeConfigVersion = 1;
static const uint16_t kRPMGaugeEepromAddress = 0;

// CAN updates arrive at ~50Hz, so most moves only span a few steps and never
// leave the slow ramp-up zone of vid6608's default table (reaches top speed
// only after 800 steps). Same start delay (stall-safe) and top speed (proven)
// as the default table, but ramps to cruise within 65 steps instead of 800.
static vid6608::AccelTable kRPMAccelTable[] = {
    {4, 3000},
    {8, 2400},
    {12, 1900},
    {16, 1500},
    {20, 1150},
    {25, 900},
    {30, 700},
    {36, 550},
    {42, 430},
    {48, 360},
    {55, 320},
    {65, 300},
};

RPMGauge::RPMGauge()
{
    pinMode(kRstPin, OUTPUT);
    digitalWrite(kRstPin, LOW);
    motor = new vid6608(kStepPin, kDirPin, kSteps);
    motor->setAccelTable(kRPMAccelTable);

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

RPMGauge::~RPMGauge()
{
    delete motor;
}

void RPMGauge::loadConfig()
{
    EEPROM.get(kRPMGaugeEepromAddress, config);
    if (config.magic != kRPMGaugeConfigMagic || config.version != kRPMGaugeConfigVersion)
    {
        DEBUGLOG_PRINTLN(F("RPMGauge: No valid EEPROM config, writing defaults"));
        config.magic = kRPMGaugeConfigMagic;
        config.version = kRPMGaugeConfigVersion;
        config.minStep = kMinimumDegree * 12;
        config.maxStep = kMaximumDegree * 12;
        EEPROM.put(kRPMGaugeEepromAddress, config);
    }
    else
    {
        DEBUGLOG_PRINTLN(F("RPMGauge: EEPROM config loaded"));
    }
    DEBUGLOG_PRINT(F("RPMGauge: minStep="));
    DEBUGLOG_PRINT(config.minStep);
    DEBUGLOG_PRINT(F(" maxStep="));
    DEBUGLOG_PRINTLN(config.maxStep);
}

void RPMGauge::saveConfig()
{
    EEPROM.put(kRPMGaugeEepromAddress, config);
    DEBUGLOG_PRINT(F("RPMGauge: config saved — minStep="));
    DEBUGLOG_PRINT(config.minStep);
    DEBUGLOG_PRINT(F(" maxStep="));
    DEBUGLOG_PRINTLN(config.maxStep);
}

RPMGauge::APIResult RPMGauge::calibrateMin()
{
    config.minStep = motor->getPosition();
    saveConfig();
    return success;
}

RPMGauge::APIResult RPMGauge::calibrateMax()
{
    config.maxStep = motor->getPosition();
    saveConfig();
    return success;
}

RPMGauge::APIResult RPMGauge::setBrightness(uint8_t brightness)
{
    analogWrite(kLightPin, brightness);
    return success;
}

RPMGauge::APIResult RPMGauge::moveNeedle(uint16_t rpm, bool calibration)
{
    if (calibration)
    {
        DEBUGLOG_PRINTLN(String(F("Calibrate RPM needle to ")) + String(rpm));
        motor->moveTo(rpm * 12); // 12 steps per degree
        return success;
    }

    if (motor->isMoving())
    {
        return success;
    }

    // Map rpm to absolute motor step, where 0 rpm = config.minStep and kMaxRPM = config.maxStep
    const float ratio = constrain(rpm / kMaxRPM, 0., 1.);
    const int range = (int)config.maxStep - (int)config.minStep; // signed widen, matches Handbrake::getHandbrakePosition() pattern — avoids uint16_t wraparound if maxStep < minStep
    const uint16_t step = static_cast<uint16_t>(config.minStep + static_cast<int>(ratio * static_cast<float>(range)));
    DEBUGLOG_PRINTLN(String(F("Move RPM needle to ")) + String(rpm) + String(F(" adjusted to step ")) + String(step));
    motor->moveTo(step);
    return success;
}

RPMGauge::APIResult RPMGauge::loop()
{
    motor->loop();
    return success;
}

bool RPMGauge::isMoving()
{
    return motor->isMoving();
}
