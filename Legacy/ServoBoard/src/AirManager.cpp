#include "AirManager.h"

AirManager *AirManager::instance = nullptr;

AirManager::AirManager() {
    instance = this;

    // Init library on channel A and Arduino type MEGA 2560
    messagePort = new SiMessagePort(SI_MESSAGE_PORT_DEVICE_ARDUINO_NANO, SI_MESSAGE_PORT_CHANNEL_N, new_message_callback);
    messagePort->DebugMessage(SI_MESSAGE_PORT_LOG_LEVEL_INFO, (String)"Initialize...");

    // Register the servos
    servos = new Servos();

    messagePort->DebugMessage(SI_MESSAGE_PORT_LOG_LEVEL_INFO, (String)"Servo driver ready");
}

AirManager::~AirManager() {
    delete messagePort;
    delete servos;  // Clean up the Servos instance
}

void AirManager::loop() {
    messagePort->Tick();
}

enum MessageId {
  kServos = 1,
};

void AirManager::new_message_callback(uint16_t message_id, struct SiMessagePortPayload* payload) {
    if (instance == NULL || payload == NULL) { return; }
    switch(message_id) {
      case kServos:
        {
          if (payload->type != SI_MESSAGE_PORT_DATA_TYPE_FLOAT) { return; }
          instance->servos->setPositions(payload->len, payload->data_float);
        }
        break;
      default:
        instance->messagePort->DebugMessage(SI_MESSAGE_PORT_LOG_LEVEL_INFO, (String)"Received unknown message: "+message_id);
    }
}
