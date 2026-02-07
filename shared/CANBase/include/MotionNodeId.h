#ifndef MOTION_ACTOR_NODE_ID_H
#define MOTION_ACTOR_NODE_ID_H  
#include <Arduino.h>

// Node_IDs for CAN messages
enum class MotionNodeId : uint8_t {
  gatewayNodeId = 0x00,
  actorPair1 = 0x01,
  actorPair2 = 0x02,
  actorPair3 = 0x03
};

static const uint8_t kActorNodeCount = 3; // Anzahl der Actor-Paare (ohne Gateway)

enum class MotionActorState {
    stopped,
    homing,
    active,
    homingFailed
};

#endif // MOTION_ACTOR_NODE_ID_H