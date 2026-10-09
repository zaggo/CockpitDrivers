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
    // Plugin -> DCU. Payload: float ias, float tas (8 bytes, host order), both in knots.
    // TAS is plumbed through for completeness; the ASI's TAS ring is mechanical,
    // so AirspeedCAN ignores it.
    SerialMessageAirspeed = 0x08,
    // Plugin -> DCU. Payload: float altitudeFt, float vsiFpm (8 bytes, host order).
    // Both are signed. One serial message feeds one CAN frame (0x102) that in turn
    // feeds two boards: VerticalSpeedCAN reads the VSI half, the altimeter board
    // the altitude half.
    SerialMessageAltimeterVsi = 0x09,
    // DCU -> Plugin. Payload: float inHg (4 bytes, host order).
    // Decoded from CAN 0x340, which carries inHg * 100 as a big-endian uint16.
    SerialMessageBaro = 0x0A,
    // Plugin -> DCU. Payload: float headingDegMag (4 bytes, host order), degrees
    // magnetic. Repacked by the DCU into CAN 0x107 as a big-endian uint16 of
    // degrees * 100.
    SerialMessageCompass = 0x0B,
    // Plugin -> DCU. Payload: HsiHeadingMessage (13 bytes, host order).
    // Repacked by the DCU into CAN 0x104 as three big-endian uint16 of
    // degrees * 100 plus the flags byte.
    SerialMessageHsiHeading = 0x0C,
    // Plugin -> DCU. Payload: HsiNavMessage (8 bytes, host order).
    // Repacked into CAN 0x105 as two big-endian int16 of dots * 1000.
    SerialMessageHsiNav = 0x0D,
    // DCU -> Plugin. Payload: HsiInputToDcuMessage (5 bytes, host order).
    // Decoded from CAN 0x341. Deltas, so the plugin accumulates; there is no
    // resend behind this and a dropped frame simply loses those clicks.
    SerialMessageHsiInput = 0x0E,
    // Plugin -> DCU. Payload: AttitudeMessage (8 bytes, host order).
    // Repacked by the DCU into CAN 0x101 as two big-endian int16 of degrees * 100.
    SerialMessageAttitude = 0x0F,
    // Plugin -> DCU. Payload: TurnCoordinatorMessage (8 bytes, host order).
    // Repacked by the DCU into CAN 0x103 as two big-endian int16 of degrees * 100.
    SerialMessageTurnCoordinator = 0x10,
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

// Message payload for Plugin > DCU, repacked into CAN 0x104.
#if defined(_MSC_VER)
    #pragma pack(push, 1)
    struct HsiHeadingMessage {
        float   headingDegMag;  // heading_electric_deg_mag_pilot
        float   obsDeg;         // hsi_obs_deg_mag_pilot
        float   hdgBugDeg;      // heading_dial_deg_mag_pilot
        uint8_t flags;          // bit0 TO, bit1 FROM, bit2 GS valid
    };
    struct HsiNavMessage {
        float cdiDots;          // nav1_hdef_dot
        float gsDots;           // nav1_vdef_dot
    };
    struct HsiInputToDcuMessage {
        uint8_t control;        // 0 = OBS, 1 = HDG
        uint8_t action;         // 0 = delta, 1 = push
        int16_t delta;          // encoder clicks, signed
        uint8_t pushState;      // 0/1
    };
    struct AttitudeMessage {
        float pitchDeg;   // pitch_vacuum_deg_pilot
        float rollDeg;    // roll_vacuum_deg_pilot
    };
    struct TurnCoordinatorMessage {
        float turnRateDeg;  // turn_rate_roll_deg_pilot
        float slipDeg;      // slip_deg
    };
    #pragma pack(pop)
#else
    struct __attribute__((packed)) HsiHeadingMessage {
        float   headingDegMag;
        float   obsDeg;
        float   hdgBugDeg;
        uint8_t flags;
    };
    struct __attribute__((packed)) HsiNavMessage {
        float cdiDots;
        float gsDots;
    };
    struct __attribute__((packed)) HsiInputToDcuMessage {
        uint8_t control;
        uint8_t action;
        int16_t delta;
        uint8_t pushState;
    };
    struct __attribute__((packed)) AttitudeMessage {
        float pitchDeg;
        float rollDeg;
    };
    struct __attribute__((packed)) TurnCoordinatorMessage {
        float turnRateDeg;
        float slipDeg;
    };
#endif

// The DCU dispatches on payload length, so these sizes are the wire contract.
// Without the packing attribute above, HsiHeadingMessage would be padded to 16
// and every frame would be rejected as malformed.
static_assert(sizeof(HsiHeadingMessage) == 13, "HsiHeadingMessage must be 13 bytes on the wire");
static_assert(sizeof(HsiNavMessage) == 8, "HsiNavMessage must be 8 bytes on the wire");
static_assert(sizeof(HsiInputToDcuMessage) == 5, "HsiInputToDcuMessage must be 5 bytes on the wire");
static_assert(sizeof(RudderToDcuMessage) == 6, "RudderToDcuMessage must be 6 bytes on the wire");
static_assert(sizeof(AttitudeMessage) == 8, "AttitudeMessage must be 8 bytes on the wire");
static_assert(sizeof(TurnCoordinatorMessage) == 8, "TurnCoordinatorMessage must be 8 bytes on the wire");
#endif // SERIAL_MESSAGE_ID_H