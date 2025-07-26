#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <Arduino.h>
#include <Wire.h>
#include <avr/pgmspace.h>

// Gyro Configuration
const uint8_t kRollPin1 = 51;
const uint8_t kRollPin2 = 50;
const uint8_t kRollPin3 = 53;
const uint8_t kRollPin4 = 52;

const uint8_t kRollZeroedPin = 2;

const uint32_t kRollTotalSteps = 4096L;
const double kRollMaxRpm = 14.0;
const double kRollMinRpm = 4;
const int16_t kRollZeroAdjustDegree = 1;

const uint8_t kPitchPin1 = 47;
const uint8_t kPitchPin2 = 46;
const uint8_t kPitchPin3 = 49;
const uint8_t kPitchPin4 = 48;

const uint8_t kPitchZeroedPin = 3;

const uint32_t kPitchTotalSteps = 4096L;
const double kPitchMaxRpm = 14.0;
const double kPitchMinRpm = 4;
const int16_t kPitchZeroAdjustDegree = -1;
const double kAdjustmentFactor = 0.76923077;

// X26.168 Configuration
const uint16_t kX25TotalSteps = 315*3;
const uint8_t kX25MotorCount = 5;
const uint8_t kX25MotorPins[kX25MotorCount][4] = {
    {22, 24, 28, 26},
    {29, 27, 23, 25},
    {30, 32, 36, 34},
    {31, 33, 37, 35},
    {45, 43, 41, 39}
};

#endif // CONFIGURATION_H
