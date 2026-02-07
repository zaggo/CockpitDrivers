#ifndef MOTION_MESSAGE_ID_H
#define MOTION_MESSAGE_ID_H

#include <Arduino.h>

// Message_IDs for CAN messages
enum class MotionMessageId : uint16_t
{
    actorPairDemand = 0x110,

    // Heartbeats (Variante 2)
    // 0x300: Gateway heartbeat (Actors überwachen den DCU)
    gatewayHeartbeat = 0x300,

    // 0x301: Actor heartbeat (Gateway überwacht Actors; nodeId im Payload)
    actorHeartbeat = 0x301,

    actorPairHome = 0x380,
    actorPairStop = 0x381,
};

// Filtering
constexpr uint32_t CAN_STD_ID(MotionMessageId id)
{
    return static_cast<uint32_t>(id) << 16;
}

#endif // MOTION_MESSAGE_ID_H