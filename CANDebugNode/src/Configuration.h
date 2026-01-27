#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <Arduino.h>

#define BENCHDEBUG 0                                           

enum ServoId {
    leftTank = 0,
    rightTank,
    servoCount
};


const uint8_t kServoPins[servoCount] = {
    4, 5
};

const uint16_t kServoMinimumDegree[servoCount] = {
    0,
    0
};

const uint16_t kServoMaximumDegree[servoCount] = {
    105,
    105
};

const uint8_t kCanIntPin = 2;
const uint8_t kCanCSPin = 10;

const uint8_t kLightPin = 3;

const float poundsPerGallon = 5.87;
const float gallonsPerKg = 2.2046226218 / poundsPerGallon;


// Exakte ID-Matches (alle 11 Bits relevant)
const uint32_t MASK_EXACT = 0x07FF0000;

#endif // CONFIGURATION_H