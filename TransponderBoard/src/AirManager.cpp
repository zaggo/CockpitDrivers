#include "AirManager.h"

AirManager *AirManager::instance = nullptr;

enum MessageId
{
  kTransponderBrightness = 1,
  kTransponderIdent = 2,
  kTransponderCode = 3,
  kTransponderMode = 4,
  kTransponderLight = 5
};

AirManager::AirManager() : messagePort(nullptr), transponder(nullptr)
{
  instance = this;

  // Init library on channel A and Arduino type MEGA 2560
  messagePort = new SiMessagePort(SI_MESSAGE_PORT_DEVICE_ARDUINO_NANO, SI_MESSAGE_PORT_CHANNEL_M, new_message_callback);
  messagePort->DebugMessage(SI_MESSAGE_PORT_LOG_LEVEL_INFO, (String) "Initialize ...");

  transponder = new Transponder();
  messagePort->DebugMessage(SI_MESSAGE_PORT_LOG_LEVEL_INFO, (String) "Transponder driver ready");

  messagePort->DebugMessage(SI_MESSAGE_PORT_LOG_LEVEL_INFO, (String) "Transponder ready");
}

AirManager::~AirManager()
{
  if (transponder != NULL)
  {
    delete transponder;
  }
  if (messagePort != NULL)
  {
    delete messagePort;
  }
}

void AirManager::loop()
{
  if (messagePort != NULL)
  {
    messagePort->Tick();
  }
  if (transponder != NULL)
  {
    transponder->tick();

    if (transponder->squawkCodeUpdated)
    {
      transponder->squawkCodeUpdated = false;
      messagePort->SendMessage(kTransponderCode, transponder->getSquawkCode());
    }
    if (transponder->identRequest)
    {
      transponder->identRequest = false;
      messagePort->SendMessage(kTransponderIdent, (uint8_t)1);
    }
    if (transponder->modeUpdated)
    {
      transponder->modeUpdated = false;
      messagePort->SendMessage(kTransponderMode, static_cast<int32_t>(transponder->getMode()));
    }
  }
}

void AirManager::new_message_callback(uint16_t message_id, struct SiMessagePortPayload *payload)
{
  if (instance == NULL || payload == NULL)
  {
    return;
  }
  switch (message_id)
  {
  case kTransponderIdent:
  {
    if (instance->transponder == NULL)
    {
      return;
    }
    if (payload->type != SI_MESSAGE_PORT_DATA_TYPE_INTEGER)
    {
      return;
    }
    bool ident = (payload->data_byte[0] != 0);
    instance->transponder->setIdent(ident);
    // instance->messagePort->DebugMessage(SI_MESSAGE_PORT_LOG_LEVEL_INFO, (String)"Transponder Ident set to: "+String(ident ? "ON" : "OFF"));
  }
  break;
  case kTransponderCode:
  {
    if (instance->transponder == NULL)
    {
      return;
    }
    if (payload->type != SI_MESSAGE_PORT_DATA_TYPE_STRING)
    {
      return;
    }
    String squawkCode = String(payload->data_string);
    instance->transponder->setSquawkCode(squawkCode);
    // instance->messagePort->DebugMessage(SI_MESSAGE_PORT_LOG_LEVEL_INFO, (String)"Transponder Squawk Code set to: "+squawkCode);
  }
  break;
  case kTransponderMode:
  {
    if (instance->transponder == NULL)
    {
      return;
    }
    if (payload->type != SI_MESSAGE_PORT_DATA_TYPE_INTEGER)
    {
      return;
    }
    uint8_t modeValue = payload->data_byte[0];
    Transponder::TransponderMode mode = static_cast<Transponder::TransponderMode>(modeValue);
    instance->transponder->setMode(mode);
    // instance->messagePort->DebugMessage(SI_MESSAGE_PORT_LOG_LEVEL_INFO, (String)"Transponder Mode set to: "+modeValue);
  }
  break;
  case kTransponderBrightness:
  {
    if (instance->transponder == NULL)
    {
      return;
    }
    if (payload->type != SI_MESSAGE_PORT_DATA_TYPE_FLOAT)
    {
      return;
    }
    uint8_t brightnessValue = static_cast<uint8_t>(roundf(payload->data_float[0] * 7.));
    if (brightnessValue > 7)
    {
      brightnessValue = 7;
    }
    if (brightnessValue == instance->transponder->getBrightness())
    {
      return; // No change
    }
    instance->transponder->setBrightness(brightnessValue);
    // instance->messagePort->DebugMessage(SI_MESSAGE_PORT_LOG_LEVEL_INFO, (String)"Transponder Brightness set to: "+brightnessValue);
  }
  break;
  case kTransponderLight:
  {
    if (instance->transponder == NULL)
    {
      return;
    }
    if (payload->type != SI_MESSAGE_PORT_DATA_TYPE_INTEGER)
    {
      return;
    }
    bool lightOn = (payload->data_byte[0] != 0);
    instance->transponder->setTransponderLight(lightOn);
    // instance->messagePort->DebugMessage(SI_MESSAGE_PORT_LOG_LEVEL_INFO, (String)"Transponder Light set to: "+String(lightOn ? "ON" : "OFF"));
  }
  break;
  default:
    instance->messagePort->DebugMessage(SI_MESSAGE_PORT_LOG_LEVEL_INFO, (String) "Received unknown message: " + message_id);
  }
}
