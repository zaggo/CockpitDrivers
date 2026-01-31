#ifndef DCUSENDER_H
#define DCUSENDER_H
#include <Arduino.h>
#include "Configuration.h"

// Message type constants for outgoing messages
static constexpr uint8_t MSG_TRANSPONDER_INPUT = 0x10;

class DCUSender {
    public:
        DCUSender();
        ~DCUSender();

        void sendTransponderInput(uint16_t code, uint8_t mode, uint8_t ident);

    private:
        void sendFrame(uint8_t type, uint8_t len, const uint8_t* payload);
};

#endif // DCUSENDER_H
