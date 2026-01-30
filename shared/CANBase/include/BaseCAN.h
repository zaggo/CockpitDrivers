#ifndef BASECAN_H
#define BASECAN_H
#include <Arduino.h>
#include "CanNodeId.h"
#include <mcp_can.h>
#include <SPI.h>
#include "CanMessageId.h"

struct CANFirmwareInfo
{
    CanNodeId nodeId;
    uint8_t fwMajor;
    uint8_t fwMinor;
};

class BaseCAN
{
public:
    BaseCAN(uint8_t csPin, uint8_t intPin, CANFirmwareInfo fwInfo);
    virtual ~BaseCAN();
    
    virtual bool begin();

    bool sendMessage(CanMessageId id, uint8_t len, byte* data);

protected:
    MCP_CAN *canBus = nullptr;
    uint8_t intPin;

    volatile bool isStarted = false;
    CANFirmwareInfo fwInfo;

    // MCP2515 /INT is active-low and stays low while RX buffers have pending frames.
    // Keep ISR minimal: just set a flag. (Do NOT touch SPI or Serial in ISR.)
    static void onCanInterrupt();
    static volatile bool canIrq;
    static BaseCAN *instance;
};

#endif // BASECAN_H