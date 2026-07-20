#include "HeartbeatDecoder.h"
#include <cstdio>
#include <cstdint>

static int g_failures = 0, g_checks = 0;
static void check(bool c, const char* w){ ++g_checks; if(!c){++g_failures; std::printf("  FAIL: %s\n", w);} }

// Feed a byte sequence; return how many complete frames were decoded, and the
// last armed value via out-param.
static int feedAll(HeartbeatDecoder& d, const uint8_t* b, int n, bool& lastArmed) {
    int frames = 0;
    for (int i = 0; i < n; ++i) {
        if (d.feed(b[i])) { ++frames; lastArmed = d.armed(); }
    }
    return frames;
}

int main() {
    // Valid armed frame.
    {
        HeartbeatDecoder d;
        const uint8_t f[] = {'H','B',0x01,0x0D};
        bool armed = false;
        int n = feedAll(d, f, 4, armed);
        check(n == 1, "armed frame decodes exactly one frame");
        check(armed == true, "armed frame -> armed() true");
    }
    // Valid disarmed frame.
    {
        HeartbeatDecoder d;
        const uint8_t f[] = {'H','B',0x00,0x0D};
        bool armed = true;
        int n = feedAll(d, f, 4, armed);
        check(n == 1, "disarmed frame decodes one frame");
        check(armed == false, "disarmed frame -> armed() false");
    }
    // Leading garbage then a valid frame.
    {
        HeartbeatDecoder d;
        const uint8_t f[] = {0x00,0xFF,'X','B','H','B',0x01,0x0D};
        bool armed = false;
        int n = feedAll(d, f, 8, armed);
        check(n == 1, "garbage-prefixed frame still decodes");
        check(armed == true, "garbage-prefixed frame -> armed true");
    }
    // Truncated frame (no CR) followed by a fresh valid frame.
    {
        HeartbeatDecoder d;
        const uint8_t f[] = {'H','B',0x01, /*no CR*/ 'H','B',0x00,0x0D};
        bool armed = true;
        int n = feedAll(d, f, 7, armed);
        check(n == 1, "truncated then fresh frame decodes once");
        check(armed == false, "recovered frame -> armed false");
    }
    // Wrong terminator byte does not complete a frame.
    {
        HeartbeatDecoder d;
        const uint8_t f[] = {'H','B',0x01,0x00};  // 0x00 instead of CR
        bool armed = false;
        int n = feedAll(d, f, 4, armed);
        check(n == 0, "wrong terminator -> no frame");
    }
    // Two back-to-back frames.
    {
        HeartbeatDecoder d;
        const uint8_t f[] = {'H','B',0x01,0x0D,'H','B',0x00,0x0D};
        bool armed = true;
        int n = feedAll(d, f, 8, armed);
        check(n == 2, "two frames decode as two");
        check(armed == false, "last frame armed value wins");
    }

    std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
