#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <Arduino.h>

#define BENCHDEBUG 0                                           

const uint8_t kCanIntPin = 2;
const uint8_t kCanCSPin = 53;

// State_IDs
enum class CanStateId : uint16_t {
  // Gateway -> Instruments
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