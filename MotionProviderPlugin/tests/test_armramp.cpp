#include "ArmRamp.h"
#include <cstdio>

static int g_failures = 0, g_checks = 0;
static void check(bool c, const char* w){ ++g_checks; if(!c){++g_failures; std::printf("  FAIL: %s\n", w);} }

int main() {
    // From Disarmed -> Arming.
    {
        ArmRamp r;   // starts Disarmed
        r.requestArm();
        check(r.state() == ArmState::Arming, "requestArm from Disarmed -> Arming");
    }
    // From Disarming -> Arming (reverses the ramp).
    {
        ArmRamp r;
        r.requestArm();                      // Arming
        r.update(1.0, 0.5, 0.5);             // ramps to Armed (blend 1)
        check(r.state() == ArmState::Armed, "reaches Armed");
        r.requestDisarm();                   // Disarming
        check(r.state() == ArmState::Disarming, "requestDisarm -> Disarming");
        r.requestArm();                      // back to Arming mid-ramp
        check(r.state() == ArmState::Arming, "requestArm from Disarming -> Arming");
    }
    // From Armed -> unchanged (no toggle).
    {
        ArmRamp r;
        r.requestArm();
        r.update(1.0, 0.5, 0.5);             // Armed
        r.requestArm();                      // idempotent
        check(r.state() == ArmState::Armed, "requestArm while Armed -> stays Armed");
    }
    // From Arming -> unchanged.
    {
        ArmRamp r;
        r.requestArm();                      // Arming
        r.requestArm();                      // idempotent
        check(r.state() == ArmState::Arming, "requestArm while Arming -> stays Arming");
    }

    std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
