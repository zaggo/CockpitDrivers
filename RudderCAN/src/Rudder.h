#ifndef RUDDER_H
#define RUDDER_H
#include <Arduino.h>
#include "Configuration.h"
#include "AdaptiveFilter.h"

struct RudderConfig {
    uint32_t magic;            // identifies a valid config record
    uint16_t version;          // struct version for future extensions
    uint16_t rudderMin;        // raw value at full left deflection
    uint16_t rudderCenter;     // raw value at neutral
    uint16_t rudderMax;        // raw value at full right deflection
    uint16_t leftBrakeMin;     // raw value with the left toe brake released
    uint16_t leftBrakeMax;     // raw value with the left toe brake fully pressed
    uint16_t rightBrakeMin;
    uint16_t rightBrakeMax;
};

// Wire-scaled axis values, ready to be packed into CanMessageId::rudder.
struct RudderState {
    int16_t  rudder;      // -1000..1000, left negative
    uint16_t leftBrake;   //     0..1000
    uint16_t rightBrake;  //     0..1000
};

struct RudderStateUpdate {
    bool changed;
    RudderState state;
};

class Rudder
{
public:
    Rudder();
    ~Rudder();

    RudderState getState();
    // Same as getState(), but flags whether any axis moved far enough since the
    // last reported state to be worth putting on the bus.
    RudderStateUpdate getStateUpdate();

    // Reads the ADCs into the axis filters. Call this on every loop() pass; it
    // rate-gates itself to kSampleIntervalMs internally.
    void sample();

    // Filtered raw values in Q6, for bench diagnostics.
    int32_t getFilteredRudderQ6() const;
    int32_t getFilteredLeftBrakeQ6() const;
    int32_t getFilteredRightBrakeQ6() const;

    // The calibration currently in effect, for bench diagnostics.
    const RudderConfig& getConfig() const;

    uint16_t getRawRudder();
    uint16_t getRawLeftBrake();
    uint16_t getRawRightBrake();

    void calibrateRudderMin();
    void calibrateRudderCenter();
    void calibrateRudderMax();
    void calibrateLeftBrakeMin();
    void calibrateLeftBrakeMax();
    void calibrateRightBrakeMin();
    void calibrateRightBrakeMax();

private:
    RudderConfig _config;
    bool _hasLastReportedState;
    RudderState _lastReportedState;

    void loadConfig();
    void saveConfig();
    uint16_t sampleAverage(uint8_t pin, uint8_t count);

    AdaptiveFilter _rudderFilter;
    AdaptiveFilter _leftBrakeFilter;
    AdaptiveFilter _rightBrakeFilter;
    uint32_t _lastSampleMs;
};

#endif // RUDDER_H
