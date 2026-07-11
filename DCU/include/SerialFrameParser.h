#ifndef SERIALFRAMEPARSER_H
#define SERIALFRAMEPARSER_H
#include <stdint.h>
#include <SerialMessageId.h>

// Parses the DCU<->plugin serial framing: 0xAA 0x55 TYPE LEN PAYLOAD...
// Pure byte-in/frame-out state machine, no Serial/hardware dependency.
class SerialFrameParser {
public:
    static const uint8_t kMaxPayload = 32;

    // Feed one byte. Returns true and fills outType/outLen/outPayload when a
    // complete frame has just been decoded. A LEN greater than kMaxPayload is
    // treated as invalid and resyncs on the next 0xAA/0x55 pair.
    bool feed(uint8_t b, MessageType* outType, uint8_t* outLen, uint8_t* outPayload) {
        switch (state) {
        case State::SyncAA:
            state = (b == 0xAA) ? State::Sync55 : State::SyncAA;
            break;

        case State::Sync55:
            state = (b == 0x55) ? State::Type : State::SyncAA;
            break;

        case State::Type:
            type = static_cast<MessageType>(b);
            state = State::Len;
            break;

        case State::Len:
            len = b;
            idx = 0;
            state = (len > kMaxPayload) ? State::SyncAA : State::Payload;
            break;

        case State::Payload:
            payload[idx++] = b;
            if (idx >= len) {
                *outType = type;
                *outLen = len;
                for (uint8_t i = 0; i < len; ++i) {
                    outPayload[i] = payload[i];
                }
                state = State::SyncAA;
                return true;
            }
            break;
        }
        return false;
    }

private:
    enum class State : uint8_t { SyncAA, Sync55, Type, Len, Payload };

    State state = State::SyncAA;
    MessageType type = static_cast<MessageType>(0);
    uint8_t len = 0;
    uint8_t payload[kMaxPayload] = {0};
    uint8_t idx = 0;
};

#endif // SERIALFRAMEPARSER_H
