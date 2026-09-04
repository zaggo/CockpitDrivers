#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <Arduino.h>
#include <CanNodeId.h>

#define BENCHDEBUG 1
#define COUPLED_MODE 0

const CanNodeId kNodeId = CanNodeId::altimeterNodeId;

// CAN (MCP2515 on hardware SPI: D11 MOSI, D12 MISO, D13 SCK).
// /CS sits on D10 deliberately. D10 is the AVR /SS pin and must be an output in
// master mode; making it the chip select means SPI.begin() drives it for us.
// Left unused it would be a floating input, and a low level there drops the SPI
// out of master mode.
const uint8_t kCanIntPin = 4;
const uint8_t kCanCSPin = 10;

// Panel light. D5 is Timer0-backed PWM; the servo library owns Timer1 and takes
// PWM on D9/D10 with it, so D5 is the one that can still dim.
const uint8_t kLightPin = 5;

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

// The 10k sensor moved off D10 for the /SS reason above: it is INPUT_PULLUP and
// active LOW, so every pass over its magnet would have killed the CAN link.
const uint8_t kHallPins[altimeterAxisCount] = {
    2, // 100s Hall sensor
    8, // 1000s Hall sensor
    7  // 10ks Hall sensor
};

const uint8_t kPotentiometerPin = A0;

const uint32_t kTotalSteps[altimeterAxisCount] = {
    4096L, 4096L, 4096L
};

// Needle zero adjust now lives in EEPROM (see include/AltimeterCalibration.h).
// These are only the factory defaults, and they are the values the AirManager
// firmware had compiled in, so a freshly flashed board behaves like the old one.
const int16_t kDefaultZeroAdjustDegree[altimeterAxisCount] = {
    5, // 100s
    2, // 1000s
    15 // 10ks
};

// Barometer defaults: full pot travel spans the usual Kollsman range.
const uint16_t kDefaultBaroLowRaw = 0;
const uint16_t kDefaultBaroLowInHg100 = 2810;
const uint16_t kDefaultBaroHighRaw = 1023;
const uint16_t kDefaultBaroHighInHg100 = 3100;

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

// Exakte ID-Matches (alle 11 Bits relevant)
const uint32_t MASK_EXACT = 0x07FF0000;

#endif // CONFIGURATION_H
