#ifndef RPMGAUGE_H
#define RPMGAUGE_H
#include <Arduino.h>
#include <vid6608.h>
#include "Configuration.h"

class RPMGauge
{
public:
    enum APIResult
    {
        success = 0
    };

public:
    RPMGauge();
    ~RPMGauge();

    APIResult moveNeedle(uint16_t rpm, bool calibration = false);
    APIResult setBrightness(uint8_t brightness);
    APIResult calibrateMin();
    APIResult calibrateMax();
    APIResult loop();
    bool isMoving();

private:
    struct Config
    {
        uint32_t magic;
        uint16_t version;
        uint16_t minStep;
        uint16_t maxStep;
    };

    void loadConfig();
    void saveConfig();

    vid6608 *motor;
    Config config;
};

#endif
