#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <Arduino.h>

#ifndef BENCHDEBUG
#define BENCHDEBUG 0          // set via build_flags in env:megaatmega2560_bench
#endif
#ifndef DEBUGLOG_ENABLE
#define DEBUGLOG_ENABLE 1     // Set to 1 to enable debug logging, 0 to disable
#endif

const uint32_t kHeartbeatInterval = 1000L; // 1 second
static const unsigned long kSerialProcessingBudgetMs = 100;
// 0 = forward every received frame immediately (plugin sends 60 Hz; per-pair
// change-dedup in processDemands() still applies). A 30 ms gate aliased against
// the 60 Hz input into irregular 17/33/50 ms forwarding gaps -> jerky motion.
static const unsigned long kDemandBatchIntervalMs = 0;

const uint8_t kStatusLedRedPin = 22;
const uint8_t kStatusLedGreenPin = 23;

const uint8_t kMode1Pin = 25; // Mode pin 1
const uint8_t kMode2Pin = 24; // Mode pin 2

const uint8_t kArmPin = 26; // Arm switch: switch between kArmPin and GND, INPUT_PULLUP

const uint32_t kUsbHeartbeatIntervalMs = 500L; // Arm heartbeat period to MotionProviderPlugin

const uint8_t kCanIntPin = 2; // MCP2515 /INT pin
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