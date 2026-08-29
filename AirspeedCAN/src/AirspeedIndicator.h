#ifndef AIRSPEEDINDICATOR_H
#define AIRSPEEDINDICATOR_H
#include <Arduino.h>
#include <vid6608.h>
#include "AirspeedCalibration.h"
#include "Configuration.h"

class AirspeedIndicator
{
public:
    enum APIResult
    {
        success = 0
    };

public:
    AirspeedIndicator();
    ~AirspeedIndicator();

    APIResult moveNeedle(float ias, bool calibration = false);
    APIResult setBrightness(uint8_t brightness);

    // Re-runs the blocking homing sweep and parks the needle on the mechanical
    // zero stop. Calibration always starts here, because every stored step
    // count is relative to that stop.
    APIResult home();

    // Records the needle's current position as the calibration point for
    // `knots` and persists the table. Returns false if the table is full.
    bool calibratePoint(uint16_t knots);

    // Factory reset: drops every calibration point but the 0kt anchor.
    APIResult wipeCalibration();

    const CalibrationTable &calibration() const { return config.table; }

    APIResult loop();
    bool isMoving();

private:
    struct Config
    {
        uint32_t magic;
        uint16_t version;
        CalibrationTable table;
    };

    void loadConfig();
    void saveConfig();

    vid6608 *motor;
    Config config;
};

#endif
