#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <Arduino.h>

#define BENCHDEBUG 0
#define DEBUGLOG_ENABLE 1

const uint8_t kCanIntPin = 48; // MCP2515 /INT pin
const uint8_t kCanCSPin = 53;

const uint8_t kCANAlarmPin = 49; // LED on if CAN error/not initialized

// Exakte ID-Matches (alle 11 Bits relevant)
const uint32_t MASK_EXACT = 0x07FF0000;

// Range match: compares the top 7 of the 11 id bits, so one filter covers a
// block of 16 consecutive ids. Same shape MotionGateway uses for its actors.
const uint32_t MASK_RANGE = 0x07F00000;

#endif // CONFIGURATION_H