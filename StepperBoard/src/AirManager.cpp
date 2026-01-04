#include "AirManager.h"

AirManager *AirManager::instance = nullptr;

enum MessageId
{
  kTurnrate = 1,
  kSideslip = 2,
  kAirSpeed = 3,
  kVerticalSpeed = 4,
  kEngineSpeedRPM = 5,
  kAttitudeIndicator = 7,
  kTransponderBrightness = 8,
  kTransponderIdent = 9,
  kTransponderCode = 10,
  kTransponderMode = 11,
  kTransponderLight = 12
};

AirManager::AirManager() : messagePort(nullptr), gyroDrive(nullptr), x25Motors(nullptr), transponder(nullptr)
{
  instance = this;

  // Init library on channel A and Arduino type MEGA 2560
  messagePort = new SiMessagePort(SI_MESSAGE_PORT_DEVICE_ARDUINO_MEGA_2560, SI_MESSAGE_PORT_CHANNEL_P, new_message_callback);
  messagePort->DebugMessage(SI_MESSAGE_PORT_LOG_LEVEL_INFO, (String) "Initialize and home motors...");

  x25Motors = new X25Motors();
  messagePort->DebugMessage(SI_MESSAGE_PORT_LOG_LEVEL_INFO, (String)kX25MotorCount + " Servos zeroed and driver ready");

  gyroDrive = new GyroDrive();
  gyroDrive->homeAllAxis();
  messagePort->DebugMessage(SI_MESSAGE_PORT_LOG_LEVEL_INFO, (String) "Attitude Indicator zeroed and driver ready");

  //transponder = new Transponder();
  //messagePort->DebugMessage(SI_MESSAGE_PORT_LOG_LEVEL_INFO, (String)"Transponder driver ready");

  messagePort->DebugMessage(SI_MESSAGE_PORT_LOG_LEVEL_INFO, (String) "Cockpit driver ready");
}

AirManager::~AirManager()
{
  if (transponder != NULL)
  {
    delete transponder;
  }
  delete x25Motors;
  delete gyroDrive;
  delete messagePort;
}

void AirManager::loop()
{
  messagePort->Tick();
  x25Motors->updateAllX25Steppers();
  gyroDrive->runAllAxes();
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
  case kAirSpeed:
  case kTurnrate:
  case kSideslip:
  case kVerticalSpeed:
  case kEngineSpeedRPM:
  {
    if (payload->type != SI_MESSAGE_PORT_DATA_TYPE_FLOAT)
    {
      return;
    }
    float relPos = payload->data_float[0];
    if (relPos < 0.0 || relPos > 1.0)
    {
      instance->messagePort->DebugMessage(SI_MESSAGE_PORT_LOG_LEVEL_INFO, (String) F("Received invalid position: ") + relPos + F(" for X25 motor: ") + message_id);
      return;
    }
    // messagePort->DebugMessage(SI_MESSAGE_PORT_LOG_LEVEL_INFO, (String)"Received position: "+relPos+" for X25 motor: "+message_id);
    // Motor-ID prüfen und Position setzen
    if (message_id > 0 && message_id <= kX25MotorCount)
    {
      instance->x25Motors->setPosition(message_id - 1, relPos);
    }
  }
  break;
  case kAttitudeIndicator:
  {
    if (payload->type != SI_MESSAGE_PORT_DATA_TYPE_FLOAT)
    {
      return;
    }
    double rollToDegree = static_cast<double>(payload->data_float[0]);
    double pitchToDegree = static_cast<double>(payload->data_float[1]) * kAdjustmentFactor;
    instance->messagePort->DebugMessage(SI_MESSAGE_PORT_LOG_LEVEL_INFO, (String) "AttitudeIndicator roll: " + rollToDegree + " pitch raw:" + static_cast<double>(payload->data_float[1]) + " pitch: " + pitchToDegree);
    instance->gyroDrive->moveToDegree(rollToDegree, pitchToDegree);
  }
  break;
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
