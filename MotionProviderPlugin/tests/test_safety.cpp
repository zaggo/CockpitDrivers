#include "SafetyLimiter.h"
#include "SafetyConfig.h"
#include "StewartKinematics.h"
#include "StewartGeometry.h"
#include "ArmRamp.h"
#include "Pose.h"
#include <cstdio>
#include <cmath>
#include <cstdint>
#include <algorithm>

static int g_failures = 0, g_checks = 0;
static void check(bool c, const char* w){ ++g_checks; if(!c){++g_failures; std::printf("  FAIL: %s\n", w);} }

int main() {
    const double dt = 1.0/60.0;

    // SafetyLimiter: step 32640 -> 65280 ramps, never exceeds velocity/accel, converges.
    {
        SafetyConfig cfg = SafetyConfig::defaults();
        SafetyLimiter lim(cfg);
        uint16_t home[6]; for(int i=0;i<6;i++) home[i]=32640;
        lim.reset(home);
        uint16_t desired[6]; for(int i=0;i<6;i++) desired[i]=65280;
        uint16_t out[6], prev[6]; for(int i=0;i<6;i++) prev[i]=32640;
        double maxStep = 0.0;
        uint16_t last = 32640;
        for (int t=0;t<600;t++) {
            lim.limit(desired, dt, out);
            double step = std::fabs((double)out[0] - (double)prev[0]);
            if (t>0) maxStep = std::max(maxStep, step);
            for(int i=0;i<6;i++) prev[i]=out[i];
            last = out[0];
        }
        // per-step move <= vMax*dt (+1 rounding)
        check(maxStep <= cfg.maxVelocity*dt + 1.5, "step within velocity limit");
        check(std::abs((int)last - 65280) <= 2, "converges to target");
    }

    // SafetyLimiter: output never leaves [0,65280] under an out-of-range demand path.
    {
        SafetyLimiter lim(SafetyConfig::defaults());
        uint16_t z[6]={0,0,0,0,0,0}; lim.reset(z);
        uint16_t hi[6]; for(int i=0;i<6;i++) hi[i]=65280;
        uint16_t out[6];
        bool inRange = true;
        for (int t=0;t<2000;t++){ lim.limit(hi, dt, out); for(int i=0;i<6;i++) if(out[i]>65280) inRange=false; }
        check(inRange, "output stays within [0,65280]");
    }

    // clampToReachable: an over-range pose becomes reachable and is scaled down.
    {
        StewartKinematics k(StewartGeometry::defaults());
        Pose big; big.pitch = 30.0f; big.roll = 30.0f;   // well outside envelope
        check(!k.solve(big).allReachable, "test pose is unreachable pre-clamp");
        Pose c = k.clampToReachable(big);
        check(k.solve(c).allReachable, "clamped pose is reachable");
        check(std::fabs(c.pitch) < std::fabs(big.pitch), "clamped pose scaled toward home");
        // A reachable pose passes through unchanged.
        Pose small; small.pitch = 1.0f;
        Pose c2 = k.clampToReachable(small);
        check(std::fabs(c2.pitch - small.pitch) < 1e-6, "reachable pose unchanged");
    }

    // ArmRamp: disarmed -> arm over ~armRampSec reaches Armed(blend 1); disarm
    // over ~disarmRampSec returns to Disarmed(blend 0); blend stays in [0,1].
    {
        ArmRamp r;
        check(r.state() == ArmState::Disarmed && r.blend() == 0.0, "starts disarmed at park");
        r.toggle();  // -> Arming
        check(r.state() == ArmState::Arming, "toggle from disarmed -> arming");
        double prev = -1.0; bool monotonic = true, inRange = true;
        for (int i = 0; i < 200; ++i) {   // 200*(1/60)=3.33s > 3s arm ramp
            r.update(1.0/60.0, 3.0, 2.0);
            if (r.blend() < prev - 1e-9) monotonic = false;
            if (r.blend() < 0.0 || r.blend() > 1.0) inRange = false;
            prev = r.blend();
        }
        check(monotonic, "arming blend monotonic up");
        check(inRange, "arming blend in [0,1]");
        check(r.state() == ArmState::Armed && std::fabs(r.blend()-1.0) < 1e-9, "reaches armed blend=1");

        r.toggle();  // -> Disarming
        check(r.state() == ArmState::Disarming, "toggle from armed -> disarming");
        for (int i = 0; i < 200; ++i) r.update(1.0/60.0, 3.0, 2.0);   // >2s disarm ramp
        check(r.state() == ArmState::Disarmed && r.blend() == 0.0, "returns to disarmed blend=0");

        // requestDisarm from armed forces a ramp-down.
        ArmRamp r2; r2.toggle(); for (int i=0;i<200;i++) r2.update(1.0/60.0,3.0,2.0);
        check(r2.state()==ArmState::Armed, "r2 armed");
        r2.requestDisarm();
        check(r2.state()==ArmState::Disarming, "requestDisarm -> disarming");
    }

    // Clip counters: a full-scale step saturates both limits on all six channels.
    {
        SafetyLimiter lim(SafetyConfig::defaults());
        uint16_t home[6]; for (int i = 0; i < 6; ++i) home[i] = 32640;
        lim.reset(home);
        uint16_t want[6]; for (int i = 0; i < 6; ++i) want[i] = 65280;
        uint16_t out[6];
        lim.limit(want, 1.0 / 60.0, out);
        check(lim.velClipCount() == 6, "full-scale step clips velocity on all 6");
        check(lim.accClipCount() == 6, "full-scale step clips acceleration on all 6");
    }

    // A held target clips nothing.
    {
        SafetyLimiter lim(SafetyConfig::defaults());
        uint16_t home[6]; for (int i = 0; i < 6; ++i) home[i] = 32640;
        lim.reset(home);
        uint16_t out[6];
        lim.limit(home, 1.0 / 60.0, out);
        check(lim.velClipCount() == 0 && lim.accClipCount() == 0,
              "held target clips nothing");
    }

    std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
