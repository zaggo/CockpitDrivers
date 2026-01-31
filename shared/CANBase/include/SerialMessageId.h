#ifndef SERIAL_MESSAGE_ID_H
#define SERIAL_MESSAGE_ID_H 

#ifdef ARDUINO
    #include <Arduino.h>
#else
    #include <cstdint>
#endif

enum class MessageType : uint8_t {
    SerialMessageFuel   = 0x01,
    SerialMessageLights = 0x02,
    SerialMessageTransponder = 0x03,


    SerialMessageTransponderInput = 0x10,
};
#endif // SERIAL_MESSAGE_ID_H