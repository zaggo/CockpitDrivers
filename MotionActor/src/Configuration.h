#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <Arduino.h>
#include <MotionNodeId.h>

#define BENCHDEBUG 0                                           

const MotionNodeId kNodeId = MotionNodeId::actorPair2;
const byte kActorAddress = 129; // Default Packet Serial address for Kangaroo is 128. Adjust if needed.
const uint32_t kKangarooBaudRate = 19200; // Default baud rate for Kangaroo. Adjust if needed.

const uint8_t kCanIntPin = 2;
const uint8_t kCanCSPin = 53;

const uint8_t kCANAlarmPin = 49; // LED on if CAN error/not initialized

// Exakte ID-Matches (alle 11 Bits relevant)
const uint32_t MASK_EXACT = 0x07FF0000;

#endif // CONFIGURATION_H