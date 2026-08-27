#include "BffEncoder.h"

void BffEncoder::encode(const uint16_t sp[6], uint8_t out[kFrameSize]) {
    out[0] = 'B';
    out[1] = 'C';
    out[2] = 0x00;                       // reserved
    for (int i = 0; i < 6; ++i) {
        out[3 + i]     = static_cast<uint8_t>((sp[i] >> 8) & 0xFF);  // MSB[i]
        out[3 + 6 + i] = static_cast<uint8_t>(sp[i] & 0xFF);         // LSB[i]
    }
    out[15] = 0x0D;                      // CR terminator
}

void BffEncoder::encodeGoto(const uint16_t t[6], uint16_t durationMs,
                            uint8_t out[kGotoFrameSize]) {
    out[0] = 'B';
    out[1] = 'G';
    for (int i = 0; i < 6; ++i) {
        out[2 + i]     = static_cast<uint8_t>((t[i] >> 8) & 0xFF);  // MSB[i]
        out[2 + 6 + i] = static_cast<uint8_t>(t[i] & 0xFF);         // LSB[i]
    }
    out[14] = static_cast<uint8_t>((durationMs >> 8) & 0xFF);
    out[15] = static_cast<uint8_t>(durationMs & 0xFF);
    out[16] = 0x0D;
}
