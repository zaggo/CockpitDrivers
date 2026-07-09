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
    APIResult loop();

private:
    vid6608 *motor;
};

#endif
