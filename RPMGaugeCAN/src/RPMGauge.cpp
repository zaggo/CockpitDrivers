#include "RPMGauge.h"
#include "DebugLog.h"

RPMGauge::RPMGauge()
{
    pinMode(kRstPin, OUTPUT);
    digitalWrite(kRstPin, LOW);
    motor = new vid6608(kStepPin, kDirPin, kSteps);

    pinMode(kLightPin, OUTPUT);
    analogWrite(kLightPin, 0);

    for (int i = 0; i < 3; i++)
    {
        delay(250);
        analogWrite(kLightPin, 255);
        delay(500);
        analogWrite(kLightPin, 0);
    }

    digitalWrite(kRstPin, HIGH);
    motor->zero();
}

RPMGauge::~RPMGauge()
{
    delete motor;
}

RPMGauge::APIResult RPMGauge::setBrightness(uint8_t brightness)
{
    analogWrite(kLightPin, brightness);
    return success;
}

RPMGauge::APIResult RPMGauge::moveNeedle(uint16_t rpm, bool calibration)
{
    uint16_t adjustedDegree;
    if (calibration)
    {
        adjustedDegree = rpm;
        DEBUGLOG_PRINTLN(String(F("Calibrate RPM needle to ")) + String(rpm));
    }
    else
    {
        if (motor->isMoving())
        {
            return success;
        }

        // Map rpm to degree, where 0 rpm = 0 degree and kMaxRPM = kMaximumDegree
        const float ratio = constrain(rpm / kMaxRPM, 0., 1.);
        const uint16_t degree = static_cast<uint16_t>(ratio * static_cast<float>(kMaximumDegree));
        adjustedDegree = max(kMinimumDegree, min(kMaximumDegree, degree));
        DEBUGLOG_PRINTLN(String(F("Move RPM needle to ")) + String(rpm) + String(F(" adjusted to ")) + String(adjustedDegree));
    }
    motor->moveTo(adjustedDegree * 12); // 12 steps per degree
    return success;
}

RPMGauge::APIResult RPMGauge::loop()
{
    motor->loop();
    return success;
}

bool RPMGauge::isMoving()
{
    return motor->isMoving();
}