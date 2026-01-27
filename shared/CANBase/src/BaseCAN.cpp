#include "BaseCAN.h"

volatile bool BaseCAN::canIrq = false;
BaseCAN *BaseCAN::instance = nullptr;

BaseCAN::BaseCAN(uint8_t csPin, uint8_t intPin, CANFirmwareInfo fwInfo)
    : intPin(intPin), fwInfo(fwInfo)
{
    canBus = new MCP_CAN(csPin);

    instance = this;
    canIrq = false;
    attachInterrupt(digitalPinToInterrupt(intPin), BaseCAN::onCanInterrupt, FALLING);
}
BaseCAN::~BaseCAN()
{
    detachInterrupt(digitalPinToInterrupt(intPin));
    instance = nullptr;
    if (canBus)
    {
        delete canBus;
        canBus = nullptr;
    }
}

bool BaseCAN::begin()
{
    return canBus->begin(MCP_STDEXT, CAN_500KBPS, MCP_8MHZ) != CAN_OK;
}

void BaseCAN::onCanInterrupt()
{
    if (instance == nullptr || instance->isStarted == false)
    {
        return;
    }

    // Keep ISR tiny: no SPI, no Serial.
    canIrq = true;
}

bool BaseCAN::sendMessage(CanMessageId id, uint8_t len, byte* data)
{
  // send data:  ID = 0x100, Standard CAN Frame, Data length = 8 bytes, 'data' = array of data bytes to send
  uint8_t sndStat = canBus->sendMsgBuf(static_cast<unsigned long>(id), 0, len, data);
  return (sndStat == CAN_OK);
} 