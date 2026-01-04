#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <Arduino.h>
#include <avr/pgmspace.h>

enum BenchMode {
    kGyroDrive = 1 << 0,
    kX25Motors = 1 << 1,
    kTransponder = 1 << 2,
    kAll = 0xff
};

#define BENCHDEBUG 0
#define BENCHDEBUG_MODE kX25Motors | kTransponder

const uint32_t kHeartbeatInterval = 1000L; // 1 second

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

// Transponder Configuration
const uint8_t kTransponderClkPin = 6;
const uint8_t kTransponderDioPin = 7;

const uint8_t kSDA = 20;
const uint8_t kSCL = 21;

const uint8_t kMCP23017Address = 0x20; // Default I2C address
const uint8_t kMCP23017InterruptPin = 18; // Pin connected to MCP23017 INT output

const uint8_t kLEDDigits = 6;

const uint32_t kPwrButtonLongPressDuration = 2000L; // 2 seconds

#endif // CONFIGURATION_H
