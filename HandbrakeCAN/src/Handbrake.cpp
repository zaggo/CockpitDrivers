#include "Handbrake.h"
#include "DebugLog.h"
#include <EEPROM.h>

// Magic: 'H','B','K','C' = HandBraKe Config
static const uint32_t kHandbrakeConfigMagic   = 0x48424B43;
static const uint16_t kHandbrakeConfigVersion = 1;
static const uint16_t kHandbrakeEepromAddress = 0;

Handbrake::Handbrake() {
    pinMode(kHandbrakePin, INPUT);
    _hasLastReportedPosition = false;
    _lastReportedPosition = 0;
    loadConfig();
}

Handbrake::~Handbrake() {
}

void Handbrake::loadConfig() {
    EEPROM.get(kHandbrakeEepromAddress, _config);
    if (_config.magic != kHandbrakeConfigMagic || _config.version != kHandbrakeConfigVersion) {
        DEBUGLOG_PRINTLN(F("Handbrake: No valid EEPROM config, writing defaults"));
        _config.magic          = kHandbrakeConfigMagic;
        _config.version        = kHandbrakeConfigVersion;
        _config.minRawPosition = 0;
        _config.maxRawPosition = 1023;
        EEPROM.put(kHandbrakeEepromAddress, _config);
    } else {
        DEBUGLOG_PRINTLN(F("Handbrake: EEPROM config loaded"));
    }
    DEBUGLOG_PRINT(F("Handbrake: minRaw="));
    DEBUGLOG_PRINT(_config.minRawPosition);
    DEBUGLOG_PRINT(F(" maxRaw="));
    DEBUGLOG_PRINTLN(_config.maxRawPosition);
}

uint16_t Handbrake::getRawPosition() {
    return analogRead(kHandbrakePin);
}

// Reads `count` samples with a short delay and returns the rounded average.
// Averaging cancels out symmetric ADC noise, giving a stable calibration point.
uint16_t Handbrake::sampleAverage(uint8_t count) {
    uint32_t sum = 0;
    for (uint8_t i = 0; i < count; i++) {
        sum += analogRead(kHandbrakePin);
        delay(5);
    }
    return (uint16_t)((sum + count / 2) / count);
}

void Handbrake::saveConfig() {
    EEPROM.put(kHandbrakeEepromAddress, _config);
    DEBUGLOG_PRINT(F("Handbrake: config saved — minRaw="));
    DEBUGLOG_PRINT(_config.minRawPosition);
    DEBUGLOG_PRINT(F(" maxRaw="));
    DEBUGLOG_PRINTLN(_config.maxRawPosition);
}

void Handbrake::calibrateMin() {
    _config.minRawPosition = sampleAverage(16);
    saveConfig();
}

void Handbrake::calibrateMax() {
    _config.maxRawPosition = sampleAverage(16);
    saveConfig();
}

uint8_t Handbrake::getHandbrakePosition() {
    uint16_t rawValue = getRawPosition();
    int range    = (int)_config.maxRawPosition - (int)_config.minRawPosition;
    int deadband = range / 20; // 5% at each end — guarantees endpoints are reachable despite noise
    int effMin   = (int)_config.minRawPosition + deadband;
    int effMax   = (int)_config.maxRawPosition - deadband;
    long mapped  = map((long)rawValue, effMin, effMax, 0, 100);
    return (uint8_t)constrain(mapped, 0, 100);
}

HandbrakePositionUpdate Handbrake::getPositionUpdate() {
    HandbrakePositionUpdate update;
    uint8_t newPosition = getHandbrakePosition();

    if (!_hasLastReportedPosition) {
        _hasLastReportedPosition = true;
        _lastReportedPosition = newPosition;
        update.changed = true;
        update.position = newPosition;
        return update;
    }

    bool changedSignificantly = abs((int)newPosition - (int)_lastReportedPosition) > 2;
    bool changedToExtreme = ((newPosition == 0 || newPosition == 100) && (newPosition != _lastReportedPosition));
    update.changed = changedSignificantly || changedToExtreme;
    update.position = newPosition;

    if (update.changed) {
        _lastReportedPosition = newPosition;
    }

    return update;
}