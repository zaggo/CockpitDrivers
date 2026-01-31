#ifndef DCUSENDER_H
#define DCUSENDER_H
#include <Arduino.h>
#include "Configuration.h"
#include <SerialMessageId.h>

class DCUSender {
    public:
        DCUSender();
        ~DCUSender();

        void sendTransponderInput(uint16_t code, uint8_t mode, uint8_t ident);

    private:
        void sendFrame(MessageType type, uint8_t len, const uint8_t* payload);
};

#endif // DCUSENDER_H
