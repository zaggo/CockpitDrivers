#ifndef CAN_MESSAGE_ID_H
#define CAN_MESSAGE_ID_H

#include <Arduino.h>

// Message_IDs for CAN messages
enum class CanMessageId : uint16_t {
  rpm = 0x106,

  odometer = 0x1F0,

  fuelLevel = 0x202,
  lights = 0x203,
  transponder = 0x201,

  // Heartbeats (Variante 2)
  // 0x300: Gateway heartbeat (Instrumente überwachen den DCU)
  gatewayHeartbeat = 0x300,

  // 0x301: Instrument heartbeat (DCU überwacht Instrumente; nodeId im Payload)
  instrumentHeartbeat = 0x301,

  // 0x303: Rudder pedals + toe brakes (Instrument -> DCU, onChange)
  // [0..1] rudder      int16,  -1000..1000 (sim/joystick/yoke_heading_ratio      * 1000)
  // [2..3] leftBrake   uint16,     0..1000 (sim/cockpit2/controls/left_brake_ratio  * 1000)
  // [4..5] rightBrake  uint16,     0..1000 (sim/cockpit2/controls/right_brake_ratio * 1000)
  // [6..7] reserved
  rudder = 0x303,

  transponderInput = 0x311,
  handbrakeStatus = 0x330
};

// Filtering
constexpr uint32_t CAN_STD_ID(CanMessageId id) {
  return static_cast<uint32_t>(id) << 16;
}

#endif // CAN_MESSAGE_ID_H