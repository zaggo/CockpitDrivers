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
    // Profiled arm/disarm move (gateway -> actor): run one internal Kangaroo
    // profile per channel to a target, speed derived from a shared duration.
    // Payload: [0]=nodeId [1..2]=act1 target BE [3..4]=act2 target BE
    //          [5..6]=duration_ms BE [7]=reserved.
    // Must stay in 0x380-0x38F (actors' RXB1 range filter).
    actorPairGoto = 0x382,
    actorCalibrationMove = 0x385,
    // Test orchestration (gateway -> actor). Deliberately inside the 0x380-0x38F
    // command block so the actors' existing RXB1 range filter passes them.
    actorTestStart = 0x386,
    actorTestAbort = 0x387,
    actorTestDumpRequest = 0x388,
    actorSaveLogicMin = 0x38a,
    actorSaveLogicMax = 0x38b,

    // Test telemetry (actor -> gateway), own 0x3A0-0x3AF block: the gateway's RXB1
    // uses mask 0x7F0 + filter 0x3A0 to accept it, actors' filters ignore it.
    testDumpHeader = 0x3A0,
    testDumpData = 0x3A1,
    testStatus = 0x3A2,
};

// Filtering
constexpr uint32_t CAN_STD_ID(MotionMessageId id)
{
    return static_cast<uint32_t>(id) << 16;
}

#endif // MOTION_MESSAGE_ID_H