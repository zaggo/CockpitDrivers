#ifndef HSIWIRE_H
#define HSIWIRE_H

#include <stdint.h>
#include <math.h>

// Wire helpers for CAN 0x104, shared by the DCU and DCUProviderPlugin - both
// resolve this directory already. Deliberately Arduino-free: the plugin is
// desktop C++, and the native Unity tests in DCU/test/test_hsi_flags exercise
// these directly (pio test -e native).

// Composes the flags byte of CAN 0x104 byte 6.
//   fromTo            sim/cockpit/radios/nav1_fromto: 0 none, 1 TO, 2 FROM
//   glideslopeValid   sim/cockpit/radios/nav1_CDI, nonzero when NAV1 is
//                     receiving an expected glide slope
// Anything other than 1 or 2 in fromTo means no usable bearing, which is the
// same as "no nav" and leaves both direction bits clear.
inline uint8_t hsiFlagsByte(int fromTo, int glideslopeValid)
{
    uint8_t flags = 0;
    if (fromTo == 1)
    {
        flags |= 0x01;
    }
    else if (fromTo == 2)
    {
        flags |= 0x02;
    }
    if (glideslopeValid != 0)
    {
        flags |= 0x04;
    }
    return flags;
}

// Folds a heading into 0..35999 hundredths of a degree. X-Plane hands out
// slightly negative values and values past 360 depending on its internal
// state; an unwrapped negative cast to uint16 would send an axis on a full
// spurious turn.
inline uint16_t hsiWrapDeg100(float degrees)
{
    float wrapped = fmodf(degrees, 360.0f);
    if (wrapped < 0.0f)
    {
        wrapped += 360.0f;
    }
    uint32_t deg100 = (uint32_t)(wrapped * 100.0f + 0.5f);
    if (deg100 > 35999)
    {
        deg100 = 0;
    }
    return (uint16_t)deg100;
}

#endif // HSIWIRE_H
