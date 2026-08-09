#include "Rudder.h"
#include "AdaptiveFilter.h"
#include "AxisMapping.h"
#include "DebugLog.h"
#include <EEPROM.h>

// Magic: 'R','U','D','C' = RUDder Config
static const uint32_t kRudderConfigMagic   = 0x52554443;
static const uint16_t kRudderConfigVersion = 1;
static const uint16_t kRudderEepromAddress = 0;

// Raw ADC counts around the calibrated center that still report 0. Rudder pedals
// have mechanical slop, so without this the aircraft would never track straight.
// Expressed in Q6, like everything else downstream of the filter.
//
// This is an absolute count, unlike the proportional 5% end deadbands in
// AxisMapping.h (span / 20): at a +-490 raw span it is 2.4% of half-travel, but
// 24% at +-50. Deliberately left absolute here and not made proportional — that
// would be a behaviour change, and the real sensor span hasn't been measured yet.
// Revisit once it has.
static const int32_t kRudderCenterDeadbandQ6 = 12 * 64;

// The axis filters run far faster than the CAN send cadence: 10 samples per
// 20ms frame. Three ADC conversions at ~112us every 2ms is about 17% CPU, which
// leaves the MCP2515 SPI traffic plenty of room.
static const uint32_t kSampleIntervalMs = 2;

// Report thresholds on the 0..1000 wire scale. Effective deadzone is +-2 units
// (the compare in getStateUpdate() is strict), so a report needs a >=3 unit
// move: about 1.4 raw LSB at a +-490 span, under half an LSB at +-150. Send
// rate stays capped at 50Hz by kMinSendIntervalMs.
static const int16_t kRudderChangeThreshold = 2;
static const int16_t kBrakeChangeThreshold  = 2;

Rudder::Rudder()
    : _rudderFilter(kDefaultAlphaMin, kDefaultSlope),
      _leftBrakeFilter(kDefaultAlphaMin, kDefaultSlope),
      _rightBrakeFilter(kDefaultAlphaMin, kDefaultSlope),
      _lastSampleMs(0)
{
    pinMode(kRudderPin, INPUT);
    pinMode(kLeftBrakePin, INPUT);
    pinMode(kRightBrakePin, INPUT);

    _hasLastReportedState = false;
    _lastReportedState.rudder     = 0;
    _lastReportedState.leftBrake  = 0;
    _lastReportedState.rightBrake = 0;

    loadConfig();
}

Rudder::~Rudder() {
}

void Rudder::loadConfig() {
    EEPROM.get(kRudderEepromAddress, _config);
    if (_config.magic != kRudderConfigMagic || _config.version != kRudderConfigVersion) {
        DEBUGLOG_PRINTLN(F("Rudder: No valid EEPROM config, writing defaults"));
        _config.magic         = kRudderConfigMagic;
        _config.version       = kRudderConfigVersion;
        _config.rudderMin     = 0;
        _config.rudderCenter  = 512;
        _config.rudderMax     = 1023;
        _config.leftBrakeMin  = 0;
        _config.leftBrakeMax  = 1023;
        _config.rightBrakeMin = 0;
        _config.rightBrakeMax = 1023;
        EEPROM.put(kRudderEepromAddress, _config);
    } else {
        DEBUGLOG_PRINTLN(F("Rudder: EEPROM config loaded"));
    }

    DEBUGLOG_PRINT(F("Rudder: rud="));
    DEBUGLOG_PRINT(_config.rudderMin);
    DEBUGLOG_PRINT('/');
    DEBUGLOG_PRINT(_config.rudderCenter);
    DEBUGLOG_PRINT('/');
    DEBUGLOG_PRINT(_config.rudderMax);
    DEBUGLOG_PRINT(F(" lBrk="));
    DEBUGLOG_PRINT(_config.leftBrakeMin);
    DEBUGLOG_PRINT('/');
    DEBUGLOG_PRINT(_config.leftBrakeMax);
    DEBUGLOG_PRINT(F(" rBrk="));
    DEBUGLOG_PRINT(_config.rightBrakeMin);
    DEBUGLOG_PRINT('/');
    DEBUGLOG_PRINTLN(_config.rightBrakeMax);
}

void Rudder::saveConfig() {
    EEPROM.put(kRudderEepromAddress, _config);
    DEBUGLOG_PRINTLN(F("Rudder: config saved"));
}

uint16_t Rudder::getRawRudder()     { return analogRead(kRudderPin); }
uint16_t Rudder::getRawLeftBrake()  { return analogRead(kLeftBrakePin); }
uint16_t Rudder::getRawRightBrake() { return analogRead(kRightBrakePin); }

void Rudder::sample()
{
    const uint32_t now = millis();
    // Unsigned subtraction, so this stays correct across the millis() rollover.
    if (_rudderFilter.seeded() && (now - _lastSampleMs) < kSampleIntervalMs) {
        return;
    }
    _lastSampleMs = now;

    _rudderFilter.update(analogRead(kRudderPin));
    _leftBrakeFilter.update(analogRead(kLeftBrakePin));
    _rightBrakeFilter.update(analogRead(kRightBrakePin));
}

int32_t Rudder::getFilteredRudderQ6() const     { return _rudderFilter.valueQ6(); }
int32_t Rudder::getFilteredLeftBrakeQ6() const  { return _leftBrakeFilter.valueQ6(); }
int32_t Rudder::getFilteredRightBrakeQ6() const { return _rightBrakeFilter.valueQ6(); }

// Reads `count` samples with a short delay and returns the rounded average.
// Averaging cancels out symmetric ADC noise, giving a stable calibration point.
uint16_t Rudder::sampleAverage(uint8_t pin, uint8_t count) {
    uint32_t sum = 0;
    for (uint8_t i = 0; i < count; i++) {
        sum += analogRead(pin);
        delay(5);
    }
    return (uint16_t)((sum + count / 2) / count);
}

void Rudder::calibrateRudderMin() {
    _config.rudderMin = sampleAverage(kRudderPin, 16);
    saveConfig();
}

void Rudder::calibrateRudderCenter() {
    _config.rudderCenter = sampleAverage(kRudderPin, 16);
    saveConfig();
}

void Rudder::calibrateRudderMax() {
    _config.rudderMax = sampleAverage(kRudderPin, 16);
    saveConfig();
}

void Rudder::calibrateLeftBrakeMin() {
    _config.leftBrakeMin = sampleAverage(kLeftBrakePin, 16);
    saveConfig();
}

void Rudder::calibrateLeftBrakeMax() {
    _config.leftBrakeMax = sampleAverage(kLeftBrakePin, 16);
    saveConfig();
}

void Rudder::calibrateRightBrakeMin() {
    _config.rightBrakeMin = sampleAverage(kRightBrakePin, 16);
    saveConfig();
}

void Rudder::calibrateRightBrakeMax() {
    _config.rightBrakeMax = sampleAverage(kRightBrakePin, 16);
    saveConfig();
}

RudderState Rudder::getState()
{
    // Rate-gated, so this is a no-op on the normal path; it only matters if
    // getState() is reached before loop() has ever sampled.
    sample();

    RudderState state;
    state.rudder = mapRudderQ6(_rudderFilter.valueQ6(),
                               rawToQ6(_config.rudderMin),
                               rawToQ6(_config.rudderCenter),
                               rawToQ6(_config.rudderMax),
                               kRudderCenterDeadbandQ6);
    state.leftBrake = mapUnipolarQ6(_leftBrakeFilter.valueQ6(),
                                    rawToQ6(_config.leftBrakeMin),
                                    rawToQ6(_config.leftBrakeMax));
    state.rightBrake = mapUnipolarQ6(_rightBrakeFilter.valueQ6(),
                                     rawToQ6(_config.rightBrakeMin),
                                     rawToQ6(_config.rightBrakeMax));
    return state;
}

RudderStateUpdate Rudder::getStateUpdate() {
    RudderStateUpdate update;
    update.state = getState();

    if (!_hasLastReportedState) {
        _hasLastReportedState = true;
        _lastReportedState = update.state;
        update.changed = true;
        return update;
    }

    const int16_t dRudder = (int16_t)(update.state.rudder - _lastReportedState.rudder);
    const int16_t dLeft   = (int16_t)((int16_t)update.state.leftBrake  - (int16_t)_lastReportedState.leftBrake);
    const int16_t dRight  = (int16_t)((int16_t)update.state.rightBrake - (int16_t)_lastReportedState.rightBrake);

    bool changed = (dRudder > kRudderChangeThreshold || -dRudder > kRudderChangeThreshold)
                || (dLeft   > kBrakeChangeThreshold  || -dLeft   > kBrakeChangeThreshold)
                || (dRight  > kBrakeChangeThreshold  || -dRight  > kBrakeChangeThreshold);

    // Endpoints must always be reported exactly, otherwise the threshold can leave
    // the sim just short of full deflection / full braking.
    changed = changed
           || (update.state.rudder != _lastReportedState.rudder
               && (update.state.rudder == 0 || update.state.rudder == 1000 || update.state.rudder == -1000))
           || (update.state.leftBrake != _lastReportedState.leftBrake
               && (update.state.leftBrake == 0 || update.state.leftBrake == 1000))
           || (update.state.rightBrake != _lastReportedState.rightBrake
               && (update.state.rightBrake == 0 || update.state.rightBrake == 1000));

    update.changed = changed;
    if (changed) {
        _lastReportedState = update.state;
    }

    return update;
}
