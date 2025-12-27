#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <Arduino.h>

#define BENCHDEBUG 0

const uint8_t kMCP23017Address = 0x20;

const uint8_t kCDIEncoderAPin = 3;
const uint8_t kCDIEncoderBPin = 4;
const uint8_t kCDIEncoderPushButtonPin = 5;
const uint8_t kCDIEncoderInterruptMask = (1 << PCINT19) | (1 << PCINT20) | (1 << PCINT21); // for PCSMSK2!

const uint8_t kCompEncoderAPin = A1;
const uint8_t kCompEncoderBPin = A2;
const uint8_t kCompEncoderPushButtonPin = A3;
const uint8_t kCompEncoderInterruptMask = (1 << PCINT9) | (1 << PCINT10) | (1 << PCINT11); // for PCSMSK1!

enum ServoId {
    vorServo = 0,
    fromToServo,
    vsi1Servo,
    vsi2Servo,
    servoCount
};

enum HSIAxis {
    cdi = 0,
    compass,
    hdg,
    hsiAxisCount
};

enum RpmKeys {
    maxRpm = 0,
    minRpm,
    rpmKeyCount
};

const uint8_t kServoPins[servoCount] = {
    9,
    11,
    10,
    12
};

const uint8_t kHallPins[hsiAxisCount] = {
    2, // CDI Hall sensor
    A0, // Compass Hall sensor
    8 // HDG Hall sensor
};

const uint32_t kTotalSteps[hsiAxisCount] = {
    4096L, 4096L, 4096L
};

const int16_t kZeroAdjustDegree[hsiAxisCount] = {
    -90,85,-11
    // 92, // CDI
    // 128, // Compass
    // -115 // HDG
};

const double kServoAdjustDegree[servoCount] = {
    80., // VOR Servo
    0., // From-To Servo
    90.-19., // VSI1 Servo
    90.-1. // VSI2 Servo
};

const double kServoAdjustRatio[servoCount] = {
    2., // VOR Servo
    1., // From-To Servo
    2., // VSI1 Servo
    -10./7.  // VSI2 Servo
};

const double kServoMinimumDegree[servoCount] = {
    -40., // VOR Servo
    -999., // From-To Servo
    -40., // VSI1 Servo
    -40.  // VSI2 Servo
};

const double kServoMaximumDegree[servoCount] = {
    40., // VOR Servo
    999., // From-To Servo
    40., // VSI1 Servo
    40.  // VSI2 Servo
};


const double kRpmLimits[hsiAxisCount][rpmKeyCount] = {
    // CDI
    {12.0, 10.0},
    // Compass
    {12.0, 10.0},
    // Bug
    {12.0, 10.0}
};

#endif // CONFIGURATION_H