#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <Arduino.h>
#include <CanNodeId.h>

#define BENCHDEBUG 0

const CanNodeId kNodeId = CanNodeId::vsiNodeId;

const uint8_t kCanIntPin = 2;
const uint8_t kCanCSPin = 10;

const uint8_t kStepPin = 7;
const uint8_t kDirPin = 8;
const uint8_t kRstPin = 9;

const uint16_t kMaximumDegree = 320;
const uint16_t kSteps = kMaximumDegree * 12; // 320 degrees at 1/12 degree steps

const uint8_t kLightPin = 3;

// The fpm-to-needle mapping lives entirely in EEPROM: a table of measured
// calibration points (see include/VerticalSpeedCalibration.h), taught on the
// bench with ho/cl/cp and interpolated linearly in between. There is no
// compiled-in climb rate range - how far the dial goes in either direction is
// whatever the lowest and highest points say. Note that the 0fpm mark is just
// another taught point, not the mechanical home stop.

// Exakte ID-Matches (alle 11 Bits relevant)
const uint32_t MASK_EXACT = 0x07FF0000;

#endif // CONFIGURATION_H
