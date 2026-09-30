#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <Arduino.h>

#define BENCHDEBUG 1

const uint8_t kMCP23017Address = 0x20;

// The OBS knob sets the course pointer.
const uint8_t kObsEncoderAPin = 3;
const uint8_t kObsEncoderBPin = 4;
const uint8_t kObsEncoderPushButtonPin = 5;
const uint8_t kObsEncoderInterruptMask = (1 << PCINT19) | (1 << PCINT20) | (1 << PCINT21); // for PCMSK2!

// The HDG knob sets the heading bug. It was called the "compass" encoder, which
// it never was - it does not touch the card.
const uint8_t kHdgEncoderAPin = A1;
const uint8_t kHdgEncoderBPin = A2;
const uint8_t kHdgEncoderPushButtonPin = A3;
const uint8_t kHdgEncoderInterruptMask = (1 << PCINT9) | (1 << PCINT10) | (1 << PCINT11); // for PCMSK1!

enum ServoId {
    cdiServo = 0,
    fromToServo,
    gs1Servo,
    gs2Servo,
    servoCount
};

enum HSIAxis {
    obs = 0,
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
    -90,85,11
    // 92, // OBS
    // 128, // Compass
    // -115 // HDG
};

const double kServoAdjustDegree[servoCount] = {
    80., // cdiServo
    0., // From-To Servo
    90.-19., // gs1Servo
    90.-1. // gs2Servo
};

const double kServoAdjustRatio[servoCount] = {
    2., // cdiServo
    1., // From-To Servo
    2., // gs1Servo
    -10./7.  // gs2Servo
};

const double kServoMinimumDegree[servoCount] = {
    -40., // cdiServo
    -999., // From-To Servo
    -40., // gs1Servo
    -40.  // gs2Servo
};

const double kServoMaximumDegree[servoCount] = {
    40., // cdiServo
    999., // From-To Servo
    40., // gs1Servo
    40.  // gs2Servo
};


const double kRpmLimits[hsiAxisCount][rpmKeyCount] = {
    // OBS
    {12.0, 10.0},
    // Compass
    {12.0, 10.0},
    // Bug
    {12.0, 10.0}
};

#endif // CONFIGURATION_H
