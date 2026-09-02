#ifndef VERTICALSPEEDINDICATOR_H
#define VERTICALSPEEDINDICATOR_H
#include <Arduino.h>
#include <vid6608.h>
#include "VerticalSpeedCalibration.h"
#include "Configuration.h"

class VerticalSpeedIndicator
{
public:
    enum APIResult
    {
        success = 0
    };

public:
    VerticalSpeedIndicator();
    ~VerticalSpeedIndicator();

    APIResult moveNeedle(float fpm, bool calibration = false);
    APIResult setBrightness(uint8_t brightness);

    // Re-runs the blocking homing sweep and parks the needle on the mechanical
    // zero stop. Calibration always starts here, because every stored step
    // count is relative to that stop. The stop is *not* the 0fpm mark - that
    // one is taught like any other point.
    APIResult home();

    // Records the needle's current position as the calibration point for
    // `fpm` (negative for descent). Returns false if the table is full.
    bool calibratePoint(int16_t fpm);

    // Factory reset: drops every calibration point, leaving the needle parked
    // on the home stop.
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
