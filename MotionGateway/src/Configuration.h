#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <Arduino.h>

#define BENCHDEBUG 0                                           

const uint8_t kStatusLedRedPin = 6;
const uint8_t kStatusLedGreenPin = 7;

const uint8_t kCanIntPin = 48; // MCP2515 /INT pin
const uint8_t kCanCSPin = 53;

// Exakte ID-Matches (alle 11 Bits relevant)
const uint32_t MASK_EXACT = 0x07FF0000;

// System State (aggregated from all actors)
// Note: MotionActorState is defined in shared/CANBase/include/MotionNodeId.h
enum class SystemState : uint8_t {
    canError = 0,      // Priority 0 (highest) - CAN bus error or not initialized
    motionError = 1,   // Priority 1 - at least one actor failed
    homing = 2,        // Priority 2 - at least one actor homing
    stopping = 3,      // Priority 3 - at least one actor stopped while others active
    stopped = 4,       // Priority 4 - all actors stopped
    active = 5         // Priority 5 (lowest) - all actors active
};

#endif // CONFIGURATION_H