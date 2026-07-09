#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <Arduino.h>
#include <CanNodeId.h>

#define BENCHDEBUG 1

const CanNodeId kNodeId = CanNodeId::rpmGaugeNodeId;

const uint8_t kCanIntPin = 2;
const uint8_t kCanCSPin = 10;

const uint8_t kStepPin = 8;
const uint8_t kDirPin = 9;
const uint8_t kRstPin = 7;

const uint16_t kMinimumDegree = 0;
const uint16_t kMaximumDegree = 320;
const uint16_t kSteps = kMaximumDegree * 12; // 315 degrees at 1/3 degree steps

const uint8_t kLightPin = 3;

const float kMaxRPM = 3500.;

// Exakte ID-Matches (alle 11 Bits relevant)
const uint32_t MASK_EXACT = 0x07FF0000;

#endif // CONFIGURATION_H
