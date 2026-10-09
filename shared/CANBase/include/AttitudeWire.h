#ifndef ATTITUDEWIRE_H
#define ATTITUDEWIRE_H

#include <stdint.h>
#include <math.h>

// Wire helper for CAN 0x101, shared by the DCU and DCUProviderPlugin - both
// resolve this directory already. Deliberately Arduino-free: the native Unity
// tests in DCU/test/test_attitude_wire exercise it (pio test -e native).

// Degrees to the int16 hundredths the frame carries. Pitch and roll are both
// legitimately negative, so this rounds half away from zero instead of the
// "+0.5" trick the unsigned fields use. Values beyond the int16 are clamped
// rather than wrapped (an unloaded aircraft can hand out anything), and a NaN
// read comes out level - lroundf(NaN) is undefined behaviour.
inline int16_t attitudeDeg100(float degrees)
{
    if (degrees != degrees) // NaN
    {
        return 0;
    }
    const float scaled = degrees * 100.0f;
    if (scaled >= 32767.0f)
    {
        return 32767;
    }
    if (scaled <= -32768.0f)
    {
        return -32768;
    }
    return (int16_t)lroundf(scaled);
}

#endif // ATTITUDEWIRE_H
