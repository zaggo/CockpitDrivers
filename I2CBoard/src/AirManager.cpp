#include "AirManager.h"

AirManager *AirManager::instance = nullptr;

AirManager::AirManager() {
    instance = this;

    // Init library on channel A and Arduino type MEGA 2560
    messagePort = new SiMessagePort(SI_MESSAGE_PORT_DEVICE_ARDUINO_NANO, SI_MESSAGE_PORT_CHANNEL_O, new_message_callback);
    messagePort->DebugMessage(SI_MESSAGE_PORT_LOG_LEVEL_INFO, (String)"Initialize I2C driver on Port O...");

    tachometer = new Tachometer();
    messagePort->DebugMessage(SI_MESSAGE_PORT_LOG_LEVEL_INFO, (String)"Tachometer initialized");
    delay(100);
    messagePort->DebugMessage(SI_MESSAGE_PORT_LOG_LEVEL_INFO, (String)"I2C driver on Port I ready");
}

AirManager::~AirManager() {
    delete messagePort;
    delete tachometer;
}

void AirManager::loop() {
    messagePort->Tick();
    tachometer->asyncTask();
}

enum MessageId {
  kTachTime = 6,
};

void AirManager::new_message_callback(uint16_t message_id, struct SiMessagePortPayload* payload) {
    if (instance == NULL || payload == NULL) { return; }
    switch(message_id) {
      case kTachTime: {
          if (payload->type != SI_MESSAGE_PORT_DATA_TYPE_FLOAT || payload->len != 6) { 
            instance->messagePort->DebugMessage(SI_MESSAGE_PORT_LOG_LEVEL_ERROR, (String)"Invalid payload for message id: "+message_id);
            return; 
          }

          // instance->messagePort->DebugMessage(SI_MESSAGE_PORT_LOG_LEVEL_INFO, (String)"TachTime: "+payload->data_float[0]+" "+payload->data_float[1]+" "+payload->data_float[2]+" "+payload->data_float[3]+" "+payload->data_float[4]+" "+payload->data_float[5]);
          instance->tachometer->displayNumber(payload->data_float);          
        }
        break;
      default:
        instance->messagePort->DebugMessage(SI_MESSAGE_PORT_LOG_LEVEL_INFO, (String)"Unknown message id: "+message_id);
        break;
    }
}
