#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <Arduino.h>

#define BENCHDEBUG 0                                           
#define COUPLED_MODE 0

const uint8_t kMCP23017Address = 0x20;

enum ServoId {
    flagServo = 0,
    servoCount
};

enum AltimeterAxis {
    hundred = 0,
    thousand,
    tenshousand,
    altimeterAxisCount
};

enum RpmKeys {
    maxRpm = 0,
    minRpm,
    rpmKeyCount
};

const uint8_t kServoPins[servoCount] = {
    9
};

const uint8_t kHallPins[altimeterAxisCount] = {
    2, // 100s Hall sensor
    8, // 1000s Hall sensor
    10 // 10ks Hall sensor
};

const uint8_t kPotentiometerPin = A0;
const uint16_t kZeroPressure = 0;
const uint16_t kHundredPercentPressure = 1023;

const uint32_t kTotalSteps[altimeterAxisCount] = {
    4096L, 4096L, 4096L
};

const int16_t kZeroAdjustDegree[altimeterAxisCount] = {
    5, // 100s Degree adjust
    2, // 1000s Degree adjust
    15  // 10ks Degree adjust
};

const double kServoAdjustDegree[servoCount] = {
    0. // Flag Servo
};

const double kServoMinimumDegree[servoCount] = {
    0. // Flag Servo
};

const double kServoMaximumDegree[servoCount] = {
    90., // Flag Servo
};

const double kRpmLimits[altimeterAxisCount][rpmKeyCount] = {
    // 100s
    {14.0, 10.0},
    // 1000s
    {14.0, 10.0},
    // 10ks
    {14.0, 10.0}
};

#endif // CONFIGURATION_H