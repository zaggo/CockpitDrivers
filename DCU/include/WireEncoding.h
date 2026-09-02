#ifndef WIREENCODING_H
#define WIREENCODING_H
#include <stdint.h>

// Big-endian 16-bit pack/unpack helpers for CAN/serial payloads.
inline void packBE16(uint8_t* dst, uint16_t value) {
    dst[0] = static_cast<uint8_t>((value >> 8) & 0xFF);
    dst[1] = static_cast<uint8_t>(value & 0xFF);
}

inline uint16_t unpackBE16(const uint8_t* src) {
    return (static_cast<uint16_t>(src[0]) << 8) | static_cast<uint16_t>(src[1]);
}

// Same for 32-bit fields (altitude on CAN 0x102 is an int32 foot count).
inline void packBE32(uint8_t* dst, uint32_t value) {
    dst[0] = static_cast<uint8_t>((value >> 24) & 0xFF);
    dst[1] = static_cast<uint8_t>((value >> 16) & 0xFF);
    dst[2] = static_cast<uint8_t>((value >> 8) & 0xFF);
    dst[3] = static_cast<uint8_t>(value & 0xFF);
}

inline uint32_t unpackBE32(const uint8_t* src) {
    return (static_cast<uint32_t>(src[0]) << 24) |
           (static_cast<uint32_t>(src[1]) << 16) |
           (static_cast<uint32_t>(src[2]) << 8) |
           static_cast<uint32_t>(src[3]);
}

#endif // WIREENCODING_H
