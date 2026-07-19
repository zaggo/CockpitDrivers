#include "SafetyMonitor.h"
#include "SafetyConfig.h"
#include "Pose.h"
#include <cstdio>
#include <cmath>

static int g_failures = 0, g_checks = 0;
static void check(bool c, const char* w){ ++g_checks; if(!c){++g_failures; std::printf("  FAIL: %s\n", w);} }

int main() {
    const double dt = 1.0/60.0;
    const SafetyConfig cfg = SafetyConfig::defaults();  // tilt 45, trans 500, hold 1.0

    // Nominal command -> no fault.
    {
        SafetyMonitor m; m.setConfig(cfg);
        Pose ok; ok.pitch = 3.0f; ok.heave = 10.0f;
        for (int i=0;i<300;i++) m.update(ok, true, false, dt);
        check(m.fault() == FaultCode::None, "nominal -> no fault");
    }

    // NaN -> latched immediately.
    {
        SafetyMonitor m; m.setConfig(cfg);
        Pose bad; bad.pitch = std::nanf("");
        m.update(bad, false, false, dt);   // finite=false
        check(m.fault() == FaultCode::Nan, "NaN latches");
        m.update(Pose{}, true, false, dt); // stays latched
        check(m.fault() == FaultCode::Nan, "fault is latched");
        m.clear();
        check(m.fault() == FaultCode::None, "clear resets");
    }

    // Serial lost while armed -> latched.
    {
        SafetyMonitor m; m.setConfig(cfg);
        m.update(Pose{}, true, true, dt);
        check(m.fault() == FaultCode::SerialLost, "serial-lost latches");
    }

    // Sustained out-of-bounds -> Runaway after holdSec; brief OOB does not.
    {
        SafetyMonitor m; m.setConfig(cfg);
        Pose oob; oob.pitch = 90.0f;   // > 45 deg
        // brief (< 1s): 30 ticks = 0.5s
        for (int i=0;i<30;i++) m.update(oob, true, false, dt);
        check(m.fault() == FaultCode::None, "brief OOB -> no fault yet");
        // back in bounds resets the accumulator
        for (int i=0;i<30;i++) m.update(Pose{}, true, false, dt);
        check(m.fault() == FaultCode::None, "in-bounds keeps clear");
        // sustained > 1s
        for (int i=0;i<80;i++) m.update(oob, true, false, dt);  // ~1.33s
        check(m.fault() == FaultCode::Runaway, "sustained OOB -> runaway");
    }

    std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
