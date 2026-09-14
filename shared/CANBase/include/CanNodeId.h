#ifndef CAN_NODE_ID_H
#define CAN_NODE_ID_H  
#include <Arduino.h>

// Node_IDs for CAN messages
enum class CanNodeId : uint8_t {
  gatewayNodeId = 0x00,
  debugNodeId = 0x01,
  fuelGaugeNodeId = 0x02,
  transponderNodeId = 0x03,
  handbrakeNodeId = 0x04,
  rpmGaugeNodeId = 0x05,
  rudderNodeId = 0x06,
  asiNodeId = 0x07,
  vsiNodeId = 0x08,
  altimeterNodeId = 0x09,
  compassNodeId = 0x0A
};
#endif // CAN_NODE_ID_H