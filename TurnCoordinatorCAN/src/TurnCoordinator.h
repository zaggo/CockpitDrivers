#ifndef TURNCOORDINATOR_H
#define TURNCOORDINATOR_H
#include <Arduino.h>
#include <vid6608.h>
#include "TurnCoordinatorCalibration.h"
#include "Configuration.h"

class TurnCoordinator
{
public:
    TurnCoordinator();
    ~TurnCoordinator();

    // Moves one needle. Normal mode maps `deg` through that needle's
    // calibration table; calibration mode drives raw dial degrees from the
    // home stop (12 steps per degree) so a mark can be found and taught.
    void moveNeedle(Needle needle, float deg, bool calibration = false);

    // Both needles from one 0x103 frame.
    void setTurnCoordinator(float turnRateDeg, float slipDeg);

    void setBrightness(uint8_t brightness);

    // Re-runs the blocking homing sweep on both motors, one after the other,
    // and parks each needle on its mechanical stop. Calibration always starts
    // here, because every stored step count is relative to that stop. The stop
    // is *not* the 0 mark - that one is taught like any other point.
    void home();

    // True once the needle has arrived at its last requested position. A
    // calibration point taken while the needle is still travelling records
    // the wrong step silently, so the bench console checks this first.
    bool isSettled(Needle needle);

    // Records the needle's current position as the calibration point for
    // `deg` (negative for left deflection). Returns false if the table is full.
    bool calibratePoint(Needle needle, int16_t deg);

    // Factory reset: drops every calibration point of both needles, leaving
    // them parked on the home stop.
    void wipeCalibration();

    const CalibrationTable &calibration(Needle needle) const { return config.tables[needle]; }

    void loop();

private:
    struct Config
    {
        uint32_t magic;
        uint16_t version;
        CalibrationTable tables[needleCount];
    };

    void loadConfig();
    void saveConfig();

    vid6608 *motors[needleCount];
    Config config;
};

#endif
