#include "AirManager.h"

AirManager *AirManager::instance = nullptr;

enum MessageId {
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
};

AirManager::AirManager() {
    instance = this;

    // Init library on channel A and Arduino type MEGA 2560
    messagePort = new SiMessagePort(SI_MESSAGE_PORT_DEVICE_ARDUINO_MEGA_2560, SI_MESSAGE_PORT_CHANNEL_P, new_message_callback);
    messagePort->DebugMessage(SI_MESSAGE_PORT_LOG_LEVEL_INFO, (String)"Initialize and home motors...");

    x25Motors = new X25Motors();
    messagePort->DebugMessage(SI_MESSAGE_PORT_LOG_LEVEL_INFO, (String)kX25MotorCount + " Servos zeroed and driver ready");

    gyroDrive = new GyroDrive();
    gyroDrive->homeAllAxis();
    messagePort->DebugMessage(SI_MESSAGE_PORT_LOG_LEVEL_INFO, (String)"Attitude Indicator zeroed and driver ready");

    transponder = new Transponder();
    messagePort->DebugMessage(SI_MESSAGE_PORT_LOG_LEVEL_INFO, (String)"Transponder driver ready");

    messagePort->DebugMessage(SI_MESSAGE_PORT_LOG_LEVEL_INFO, (String)"Copit driver ready");
}

AirManager::~AirManager() {
    delete messagePort;
    delete gyroDrive;
    delete x25Motors;
    delete transponder;
}

void AirManager::loop() {
    messagePort->Tick();
    x25Motors->updateAllX25Steppers();
    gyroDrive->runAllAxes();
    transponder->tick();

    if (transponder->squawkCodeUpdated) {
      transponder->squawkCodeUpdated = false;
      messagePort->SendMessage(kTransponderCode, transponder->getSquawkCode());
      messagePort->SendMessage(kTransponderMode, (uint8_t)transponder->getMode());
    } 
    if (transponder->identRequest) {
      transponder->identRequest = false;
      messagePort->SendMessage(kTransponderIdent, (uint8_t)1);
    }
}

void AirManager::new_message_callback(uint16_t message_id, struct SiMessagePortPayload* payload) {
    if (instance == NULL || payload == NULL) { return; }
    switch(message_id) {
      case kAirSpeed:
      case kTurnrate:
      case kSideslip:
      case kVerticalSpeed:
      case kEngineSpeedRPM: {
          if (payload->type != SI_MESSAGE_PORT_DATA_TYPE_FLOAT) { return; }
          float relPos = payload->data_float[0];
          if (relPos < 0.0 || relPos>1.0) { 
            instance->messagePort->DebugMessage(SI_MESSAGE_PORT_LOG_LEVEL_INFO, (String)"Received invalid position: "+relPos+" for X25 motor: "+message_id);
            return;
          }
          // messagePort->DebugMessage(SI_MESSAGE_PORT_LOG_LEVEL_INFO, (String)"Received position: "+relPos+" for X25 motor: "+message_id);
          // Motor-ID prüfen und Position setzen
          if (message_id > 0 && message_id <= kX25MotorCount) {
            instance->x25Motors->setPosition(message_id - 1, relPos);
          }
        }
        break;
      case kAttitudeIndicator: {
          if (payload->type != SI_MESSAGE_PORT_DATA_TYPE_FLOAT) { return; }
          double rollToDegree = static_cast<double>(payload->data_float[0]);
          double pitchToDegree = static_cast<double>(payload->data_float[1]) * kAdjustmentFactor;
          instance->messagePort->DebugMessage(SI_MESSAGE_PORT_LOG_LEVEL_INFO, (String)"AttitudeIndicator roll: "+rollToDegree+" pitch raw:"+static_cast<double>(payload->data_float[1])+" pitch: "+pitchToDegree);
          instance->gyroDrive->moveToDegree(rollToDegree, pitchToDegree);
        }
        break;
      case kTransponderIdent: {
          if (payload->type != SI_MESSAGE_PORT_DATA_TYPE_INTEGER) { return; }
          bool ident = (payload->data_byte[0] != 0);
          instance->transponder->setIdent(ident);
          instance->messagePort->DebugMessage(SI_MESSAGE_PORT_LOG_LEVEL_INFO, (String)"Transponder Ident set to: "+String(ident ? "ON" : "OFF"));
        }
        break;
      case kTransponderCode: {
          if (payload->type != SI_MESSAGE_PORT_DATA_TYPE_STRING) { return; }
          String squawkCode = String(payload->data_string);
          instance->transponder->setSquawkCode(squawkCode);
          instance->messagePort->DebugMessage(SI_MESSAGE_PORT_LOG_LEVEL_INFO, (String)"Transponder Squawk Code set to: "+squawkCode);
      }
        break;
      case kTransponderMode: {
          if (payload->type != SI_MESSAGE_PORT_DATA_TYPE_INTEGER) { return; }
          uint8_t modeValue = payload->data_byte[0];
          Transponder::TransponderMode mode = static_cast<Transponder::TransponderMode>(modeValue);
          instance->transponder->setMode(mode);
          instance->messagePort->DebugMessage(SI_MESSAGE_PORT_LOG_LEVEL_INFO, (String)"Transponder Mode set to: "+modeValue);
        }
        break;
      case kTransponderBrightness: {
          if (payload->type != SI_MESSAGE_PORT_DATA_TYPE_FLOAT) { return; }
          uint8_t brightnessValue = static_cast<uint8_t>(roundf(payload->data_float[0] * 7.));
          instance->transponder->setBrightness(brightnessValue);
          instance->messagePort->DebugMessage(SI_MESSAGE_PORT_LOG_LEVEL_INFO, (String)"Transponder Brightness set to: "+brightnessValue);
        }
        break;
      default:
        instance->messagePort->DebugMessage(SI_MESSAGE_PORT_LOG_LEVEL_INFO, (String)"Received unknown message: "+message_id);
    }
}
