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
};

// Message Payload for Transponder > DCU
enum TransponderToDCUCommand : uint8_t {
    TransponderToDcuCommandSetCode = 0x01 << 0,
    TransponderToDcuCommandSetMode = 0x01 << 1,
    TransponderToDcuCommandIdent = 0x01 << 2,
};

struct __attribute__((packed)) TransponderToDcuMessage {
    TransponderToDCUCommand  command; // Command identifier
    uint16_t code;
    uint8_t  mode;
};
#endif // SERIAL_MESSAGE_ID_H