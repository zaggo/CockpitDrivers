#include "FuelGauge.h"
#include "DebugLog.h"

FuelGauge::FuelGauge()
{
    for (int servoId = 0; servoId < servoCount; servoId++)
    {
        servos[servoId] = new Servo();
        servos[servoId]->attach(kServoPins[servoId]);
    }

    pinMode(kLightPin, OUTPUT);
    analogWrite(kLightPin, 0);

    for (int i = 0; i<3; i++) {
        delay(250);
        analogWrite(kLightPin, 255);
        delay(500);
        analogWrite(kLightPin, 0);
    }
}

FuelGauge::~FuelGauge()
{
    for (int servoId = 0; servoId < servoCount; servoId++)
    {
        servos[servoId]->detach();
        delete servos[servoId];
    }
}

FuelGauge::APIResult FuelGauge::setBrightness(uint8_t brightness) {
    analogWrite(kLightPin, brightness);
    return success;
}

FuelGauge::APIResult FuelGauge::moveServo(ServoId id, float fuelKg, bool calibration)
{
    if (id >= ServoId::servoCount)
    {
        return invalidId;
    }

    uint16_t adjustedDegree;
    if (calibration)
    {
        adjustedDegree = static_cast<uint16_t>(fuelKg);
        DEBUGLOG_PRINTLN(String(F("Calibrate ")) + servoName(id) + String(F(" to ")) + String(fuelKg));
    }
    else
    {
        const float gallons= static_cast<uint32_t>(fuelKg * gallonsPerKg);
        const uint16_t degree = static_cast<uint16_t>(gallons/40. * static_cast<float>(kServoMaximumDegree[id]));
        const uint16_t clampedDegree = max(kServoMinimumDegree[id], min(kServoMaximumDegree[id], degree));
        if (id == leftTank) {
            adjustedDegree = clampedDegree;
        } else {
            adjustedDegree = kServoMaximumDegree[id] - clampedDegree;
        }
        DEBUGLOG_PRINTLN(String(F("Move ")) + servoName(id) + String(F(" to ")) + String(gallons) + String(F(" adjusted to ")) + String(adjustedDegree));
    }
    servos[id]->write(adjustedDegree);
    return success;
}

String FuelGauge::servoName(ServoId id)
{
    switch (id)
    {
    case leftTank:
        return "Left Tank Servo";
    case rightTank:
        return "Right Tank Servo";
    default:
        return "Unknown";
    }
}