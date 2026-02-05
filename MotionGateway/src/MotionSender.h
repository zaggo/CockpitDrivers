#ifndef MOTIONSENDER_H
#define MOTIONSENDER_H
#include <Arduino.h>
#include "Configuration.h"
#include <SerialMessageId.h>

class MotionSender {
    public:
        MotionSender();
        ~MotionSender();

        void sendFrame(MessageType type, uint8_t len, const uint8_t* payload);
};

#endif // MOTIONSENDER_H
