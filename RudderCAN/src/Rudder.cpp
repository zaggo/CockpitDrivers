#include "Rudder.h"
#include "DebugLog.h"
#include <EEPROM.h>

// Magic: 'R','U','D','C' = RUDder Config
static const uint32_t kRudderConfigMagic   = 0x52554443;
static const uint16_t kRudderConfigVersion = 1;
static const uint16_t kRudderEepromAddress = 0;

// Raw ADC counts around the calibrated center that still report 0. Rudder pedals
// have mechanical slop and the ADC drifts a couple of LSB, so without this the
// aircraft would never track straight.
static const long kRudderCenterDeadbandRaw = 12;

// Report thresholds on the 0..1000 wire scale. One ADC LSB is roughly 2 units of
// a full 0..1000 half-travel, so 8 swallows noise without feeling sticky.
static const int16_t kRudderChangeThreshold = 8;
static const int16_t kBrakeChangeThreshold  = 8;

// Maps one half of a travel (from `center` to `end`) onto 0..1000. All offsets are
// derived from the signed (end - center) delta, so an inverted potentiometer —
// where the raw value falls as the pedal is pushed — works without special casing.
static uint16_t mapHalf(long raw, long center, long end, long centerDeadband)
{
    const long span = end - center;
    if (span == 0) {
        return 0;
    }
    const long from = center + ((span > 0) ? centerDeadband : -centerDeadband);
    const long to   = end - span / 20; // 5% end deadband, guarantees the endpoint is reachable
    if (from == to) {
        return 0;
    }
    return (uint16_t)constrain(map(raw, from, to, 0L, 1000L), 0L, 1000L);
}

// Maps a unidirectional axis (brake) between two calibration points onto 0..1000.
static uint16_t mapUnipolar(long raw, long lo, long hi)
{
    const long span = hi - lo;
    if (span == 0) {
        return 0;
    }
    const long deadband = span / 20; // 5% at each end
    const long from = lo + deadband;
    const long to   = hi - deadband;
    if (from == to) {
        return 0;
    }
    return (uint16_t)constrain(map(raw, from, to, 0L, 1000L), 0L, 1000L);
}

Rudder::Rudder() {
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

int16_t Rudder::mapRudder(uint16_t raw) const {
    const long v = (long)raw;
    const long center = (long)_config.rudderCenter;

    if (v - center <= kRudderCenterDeadbandRaw && center - v <= kRudderCenterDeadbandRaw) {
        return 0;
    }

    // Which half of the travel are we on? Decided against the signed direction of
    // the max endpoint, so it stays correct for inverted wiring too.
    const bool towardsMax = ((long)_config.rudderMax > center) ? (v > center) : (v < center);

    if (towardsMax) {
        return (int16_t)mapHalf(v, center, (long)_config.rudderMax, kRudderCenterDeadbandRaw);
    }
    return (int16_t)-(int16_t)mapHalf(v, center, (long)_config.rudderMin, kRudderCenterDeadbandRaw);
}

RudderState Rudder::getState() {
    RudderState state;
    state.rudder     = mapRudder(getRawRudder());
    state.leftBrake  = mapUnipolar(getRawLeftBrake(),  (long)_config.leftBrakeMin,  (long)_config.leftBrakeMax);
    state.rightBrake = mapUnipolar(getRawRightBrake(), (long)_config.rightBrakeMin, (long)_config.rightBrakeMax);
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
