#ifndef CAN_MESSAGE_ID_H
#define CAN_MESSAGE_ID_H

#include <Arduino.h>

// Message_IDs for CAN messages
enum class CanMessageId : uint16_t {
  fuelLevel = 0x202,
  lights = 0x203,
  transponder = 0x201,

  // Heartbeats (Variante 2)
  // 0x300: Gateway heartbeat (Instrumente überwachen den DCU)
  gatewayHeartbeat = 0x300,

  // 0x301: Instrument heartbeat (DCU überwacht Instrumente; nodeId im Payload)
  instrumentHeartbeat = 0x301,

  transponderInput = 0x311,
};

// Filtering
constexpr uint32_t CAN_STD_ID(CanMessageId id) {
  return static_cast<uint32_t>(id) << 16;
}

#endif // CAN_MESSAGE_ID_H