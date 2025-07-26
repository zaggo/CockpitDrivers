#include "AirManager.h"

AirManager *AirManager::instance = nullptr;

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

    messagePort->DebugMessage(SI_MESSAGE_PORT_LOG_LEVEL_INFO, (String)"Copit driver ready");
}

AirManager::~AirManager() {
    delete messagePort;
    delete gyroDrive;
    delete x25Motors;
}

void AirManager::loop() {
    messagePort->Tick();
    x25Motors->updateAllX25Steppers();
    gyroDrive->runAllAxes();
}

enum MessageId {
  kTurnrate = 1,
  kSideslip = 2,
  kAirSpeed = 3,
  kVerticalSpeed = 4,
  kEngineSpeedRPM = 5,
  kAttitudeIndicator = 7
};

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
      default:
        instance->messagePort->DebugMessage(SI_MESSAGE_PORT_LOG_LEVEL_INFO, (String)"Received unknown message: "+message_id);
    }
}
