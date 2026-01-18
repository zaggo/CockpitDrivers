#ifndef FUELGAUGE_H
#define FUELGAUGE_H
#include <Arduino.h>
#include <Servo.h>
#include "Configuration.h"

class FuelGauge
{
public:
    enum APIResult
    {
        success = 0,
        invalidId
    };

public:
    FuelGauge();
    ~FuelGauge();

    APIResult moveServo(ServoId id, float gallons, bool calibration = false);
    APIResult setBrightness(uint8_t brightness);

private:
    Servo *servos[servoCount];
    String servoName(ServoId id);
};

#endif