#ifndef CAN_NODE_ID_H
#define CAN_NODE_ID_H  
#include <Arduino.h>

// Node_IDs for CAN messages
enum class CanNodeId : uint8_t {
  gatewayNodeId = 0x00,
  debugNodeId = 0x01,
  fuelGaugeNodeId = 0x02,
  transponderNodeId = 0x03
};
#endif // CAN_NODE_ID_H