#include "AirManager.h"

AirManager *AirManager::instance = nullptr;

enum MessageId
{
    kHSI = 1,
    kHome = 2,
    kSetInitialDegrees = 3,
    kSetDegreesCDI = 4,
    kSetDegreesCompass = 5
};

AirManager::AirManager()
{
    // Init library on channel A and Arduino type MEGA 2560
    messagePort = new SiMessagePort(SI_MESSAGE_PORT_DEVICE_ARDUINO_NANO, SI_MESSAGE_PORT_CHANNEL_H, new_message_callback);
    messagePort->DebugMessage(SI_MESSAGE_PORT_LOG_LEVEL_INFO, (String) "Initialize and home motors...");

    hsi = new HSI();
    // messagePort->DebugMessage(SI_MESSAGE_PORT_LOG_LEVEL_INFO, (String) "HSIDrive initialized");

    messagePort->DebugMessage(SI_MESSAGE_PORT_LOG_LEVEL_INFO, (String) "HSI driver v1.0.0 ready");
    instance = this;
}

AirManager::~AirManager()
{
    delete messagePort;
    delete hsi;
}

void AirManager::loop()
{
    bool wasHomed = hsi != NULL && hsi->isHomed;
    messagePort->Tick();
    if (hsi != NULL)
    {
        hsi->loop();
        if (!wasHomed && hsi->isHomed)
        {
            messagePort->SendMessage(kHome);
        }
        float currentSetDegCDI = static_cast<float>(hsi->getCdiEncoder());
        if (setDegCDI != currentSetDegCDI)
        {
            setDegCDI = currentSetDegCDI;
            messagePort->SendMessage(kSetDegreesCDI, currentSetDegCDI);
        }
        float currentSetDegCompass = static_cast<float>(hsi->getCompEncoder());
        if (setDegCompass != currentSetDegCompass)
        {
            setDegCompass = currentSetDegCompass;
            messagePort->SendMessage(kSetDegreesCompass, currentSetDegCompass);
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
    case kHSI:
    {
        if (payload->type != SI_MESSAGE_PORT_DATA_TYPE_FLOAT || payload->len != 6)
        {
            instance->messagePort->DebugMessage(SI_MESSAGE_PORT_LOG_LEVEL_ERROR, (String) "Invalid payload for message id: " + message_id);
            return;
        }
        if (instance->hsi == NULL || instance->hsi->isHomed == false)
        {
            instance->messagePort->DebugMessage(SI_MESSAGE_PORT_LOG_LEVEL_ERROR, (String) "HSI is not homed yet. Please home the HSI first.");
            return;
        }

        double cdiDegree = static_cast<double>(payload->data_float[0]);
        double compassDegree = static_cast<double>(payload->data_float[1]);
        double hdgDegree = static_cast<double>(payload->data_float[2]);
        double vorOffset = static_cast<double>(payload->data_float[3]);
        double vsiOffset = static_cast<double>(payload->data_float[4]);
        double fromTo = static_cast<double>(payload->data_float[5]);

        HSI::FromTo fromToValue;
        if (fromTo < 0.4)
        {
            fromToValue = HSI::FromTo::noNav;
        }
        else if (fromTo < 1.4)
        {
            fromToValue = HSI::FromTo::to;
        }
        else if (fromTo == 2.4)
        {
            fromToValue = HSI::FromTo::from;
        }
        else
        {
            instance->messagePort->DebugMessage(SI_MESSAGE_PORT_LOG_LEVEL_ERROR, (String) "Received invalid value for FromTo: " + fromTo);
            fromToValue = HSI::FromTo::noNav;
        }

        if (instance->hsi == NULL)
        {
            instance->messagePort->DebugMessage(SI_MESSAGE_PORT_LOG_LEVEL_ERROR, (String) "HSI instance is NULL");
            return;
        }
        instance->hsi->moveToDegree(cdiDegree, compassDegree, hdgDegree, vorOffset, fromToValue, vsiOffset);
        break;
    }
    case kHome:
        if (instance->hsi != NULL)
        {
            instance->hsi->homeAllAxis();
            instance->messagePort->DebugMessage(SI_MESSAGE_PORT_LOG_LEVEL_INFO, (String) "All axis homed");

            instance->hsi->moveServo(fromToServo, 0);
            instance->hsi->moveServo(vsi1Servo, 0);
            instance->hsi->moveServo(vsi2Servo, 0);
            instance->hsi->moveServo(vorServo, 0);
            // instance->messagePort->DebugMessage(SI_MESSAGE_PORT_LOG_LEVEL_INFO, (String) "Servos homed");
        }
        break;
    case kSetInitialDegrees:
        if (payload->type != SI_MESSAGE_PORT_DATA_TYPE_FLOAT || payload->len != 2)
        {
            instance->messagePort->DebugMessage(SI_MESSAGE_PORT_LOG_LEVEL_ERROR, (String) "Invalid payload for message id: " + message_id);
            return;
        }
        instance->setDegCDI = static_cast<double>(payload->data_float[0]);
        instance->setDegCompass = static_cast<double>(payload->data_float[1]);
        instance->messagePort->DebugMessage(SI_MESSAGE_PORT_LOG_LEVEL_INFO, (String) "Initial degrees set: CDI=" + instance->setDegCDI + ", Compass=" + instance->setDegCompass);
        break;
    default:
        instance->messagePort->DebugMessage(SI_MESSAGE_PORT_LOG_LEVEL_INFO, (String) "Received unknown message: " + message_id);
    }
}