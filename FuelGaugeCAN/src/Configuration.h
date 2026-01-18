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

// State_IDs
enum class CanStateId : uint16_t {
  fuelLevel = 0x202,
  lights = 0x203,

  // Heartbeats (Variante 2)
  // 0x300: Gateway heartbeat (Instrumente überwachen den DCU)
  gatewayHeartbeat = 0x300,
  // 0x301: Instrument heartbeat (DCU überwacht Instrumente; nodeId im Payload)
  instrumentHeartbeat = 0x301
};

// Filtering
constexpr uint32_t CAN_STD_ID(CanStateId id) {
  return static_cast<uint32_t>(id) << 16;
}

// Exakte ID-Matches (alle 11 Bits relevant)
const uint32_t MASK_EXACT = 0x07FF0000;

#endif // CONFIGURATION_H