#include "BffEncoder.h"
#include <cstdio>
#include <cstdint>

static int g_failures = 0, g_checks = 0;
static void check(bool c, const char* w){ ++g_checks; if(!c){++g_failures; std::printf("  FAIL: %s\n", w);} }

int main() {
    uint16_t sp[6] = {0x1234, 0x00FF, 0xFF00, 32640, 0, 65280};
    uint8_t f[BffEncoder::kFrameSize];
    BffEncoder::encode(sp, f);

    check(BffEncoder::kFrameSize == 16, "frame is 16 bytes");
    check(f[0] == 'B' && f[1] == 'C', "starts with BC");
    check(f[2] == 0x00, "reserved byte 0");
    check(f[15] == 0x0D, "CR terminator");

    // Round-trip decode exactly as MotionGateway::handleBFFFrame does.
    for (int i = 0; i < 6; ++i) {
        uint16_t demand = static_cast<uint16_t>((f[3 + i] << 8) | f[3 + 6 + i]);
        check(demand == sp[i], "setpoint round-trips through BFF frame");
    }
    std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
