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
    SerialMessageHandbrake = 0x04,
    SerialMessageRPM = 0x05,
    SerialMessageOdometer = 0x06,
    SerialMessageRudder = 0x07,
};

// Message Payload for Transponder > DCU
enum class TransponderToDCUCommand : uint8_t {
    TransponderToDcuCommandSetCode = 0x01 << 0,
    TransponderToDcuCommandSetMode = 0x01 << 1,
    TransponderToDcuCommandIdent = 0x01 << 2,
};

#if defined(_MSC_VER)
    #pragma pack(push, 1)
    struct TransponderToDcuMessage {
        TransponderToDCUCommand  command; // Command identifier
        uint16_t code;
        uint8_t  mode;
    };
    #pragma pack(pop)
#else
    struct __attribute__((packed)) TransponderToDcuMessage {
        TransponderToDCUCommand  command; // Command identifier
        uint16_t code;
        uint8_t  mode;
    };
#endif

// Message Payload for Rudder > DCU > Plugin.
// Host byte order (both ends are little-endian), unlike the big-endian CAN frame
// this is decoded from — DCU::updateRudder() does the conversion via unpackBE16.
#if defined(_MSC_VER)
    #pragma pack(push, 1)
    struct RudderToDcuMessage {
        int16_t  rudder;      // -1000..1000, left negative (yoke_heading_ratio * 1000)
        uint16_t leftBrake;   //     0..1000 (left_brake_ratio  * 1000)
        uint16_t rightBrake;  //     0..1000 (right_brake_ratio * 1000)
    };
    #pragma pack(pop)
#else
    struct __attribute__((packed)) RudderToDcuMessage {
        int16_t  rudder;      // -1000..1000, left negative (yoke_heading_ratio * 1000)
        uint16_t leftBrake;   //     0..1000 (left_brake_ratio  * 1000)
        uint16_t rightBrake;  //     0..1000 (right_brake_ratio * 1000)
    };
#endif
#endif // SERIAL_MESSAGE_ID_H