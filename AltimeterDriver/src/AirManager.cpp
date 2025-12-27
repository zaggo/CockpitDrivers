#include "AirManager.h"

AirManager *AirManager::instance = nullptr;

enum MessageId
{
    kAltimeter = 1,
    kHome = 2,
    kPressure = 3
};

AirManager::AirManager()
{
    // Init library on channel A and Arduino type MEGA 2560
    messagePort = new SiMessagePort(SI_MESSAGE_PORT_DEVICE_ARDUINO_NANO, SI_MESSAGE_PORT_CHANNEL_L, new_message_callback);
    messagePort->DebugMessage(SI_MESSAGE_PORT_LOG_LEVEL_INFO, (String) "Initialize and home motors...");

    altimeter = new Altimeter();
    // messagePort->DebugMessage(SI_MESSAGE_PORT_LOG_LEVEL_INFO, (String) "HSIDrive initialized");
    currentPressure = altimeter->fetchPressureRatio();

    messagePort->DebugMessage(SI_MESSAGE_PORT_LOG_LEVEL_INFO, (String) "Altimeter driver v1.0.0 ready");
    instance = this;
}

AirManager::~AirManager()
{
    delete messagePort;
    delete altimeter;
}

void AirManager::loop()
{
    bool wasHomed = altimeter != NULL && altimeter->isHomed;
    messagePort->Tick();
    if (altimeter != NULL)
    {
        altimeter->loop();
        if (altimeter->isHomed && round(currentPressure*100.0) != round(altimeter->fetchPressureRatio()*100.))
        {
            currentPressure = altimeter->fetchPressureRatio();
            messagePort->SendMessage(kPressure, currentPressure);
        }
        if (!wasHomed && altimeter->isHomed)
        {
            messagePort->SendMessage(kHome);
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
    case kAltimeter:
    {
        if (payload->type != SI_MESSAGE_PORT_DATA_TYPE_FLOAT || payload->len != 1)
        {
            instance->messagePort->DebugMessage(SI_MESSAGE_PORT_LOG_LEVEL_ERROR, (String) "Invalid payload for message id: " + message_id);
            return;
        }
        if (instance->altimeter == NULL || instance->altimeter->isHomed == false)
        {
            instance->messagePort->DebugMessage(SI_MESSAGE_PORT_LOG_LEVEL_ERROR, (String) "Altimeter is not homed yet. Please home the Altimeter first.");
            return;
        }

        double heightInFeet = static_cast<double>(payload->data_float[0]);
        if (instance->altimeter == NULL)
        {
            instance->messagePort->DebugMessage(SI_MESSAGE_PORT_LOG_LEVEL_ERROR, (String) "Altimeter instance is NULL");
            return;
        }
        instance->altimeter->moveToHeight(heightInFeet);
        break;
    }
    case kHome:
        if (instance->altimeter != NULL)
        {
            if (instance->altimeter->homeAllAxis() == Altimeter::success) {
                instance->messagePort->DebugMessage(SI_MESSAGE_PORT_LOG_LEVEL_INFO, (String) "Altimeter homing successful");
            } else {
                instance->messagePort->DebugMessage(SI_MESSAGE_PORT_LOG_LEVEL_ERROR, (String) "Altimeter homing failed: " + instance->altimeter->errorName(instance->altimeter->homeAllAxis()));
            }
        }
        break;
    default:
        instance->messagePort->DebugMessage(SI_MESSAGE_PORT_LOG_LEVEL_INFO, (String) "Received unknown message: " + message_id);
    }
}