#ifndef DCUSENDER_H
#define DCUSENDER_H
#include <Arduino.h>
#include "Configuration.h"
#include <SerialMessageId.h>

class DCUSender {
    public:
        DCUSender();
        ~DCUSender();

        void sendFrame(MessageType type, uint8_t len, const uint8_t* payload);
};

#endif // DCUSENDER_H
