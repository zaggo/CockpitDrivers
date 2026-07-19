#include "EffectsLayer.h"
#include "EffectsConfig.h"
#include "MotionCues.h"
#include <cstdio>
#include <cmath>

static int g_failures = 0, g_checks = 0;
static void check(bool c, const char* w){ ++g_checks; if(!c){++g_failures; std::printf("  FAIL: %s\n", w);} }

int main() {
    const double dt = 1.0/60.0;

    // Airborne, no events -> zero offset.
    {
        EffectsLayer e(EffectsConfig::defaults());
        MotionCues air; air.onGround = false; air.groundspeed = 60.0f;
        double maxAbs = 0.0;
        for (int i=0;i<120;i++){ Pose p = e.update(air, dt); maxAbs = std::max(maxAbs, std::fabs((double)p.heave)); }
        check(maxAbs < 1e-6, "airborne -> no effect");
    }

    // Touchdown edge -> bump fires then decays toward zero.
    {
        EffectsLayer e(EffectsConfig::defaults());
        MotionCues air; air.onGround = false;
        e.update(air, dt);
        MotionCues gnd; gnd.onGround = true; gnd.groundspeed = 0.0f;
        double early = 0.0;
        for (int i=0;i<20;i++){ Pose p = e.update(gnd, dt); early = std::max(early, std::fabs((double)p.heave)); }
        double late = 0.0;
        for (int i=0;i<120;i++){ Pose p = e.update(gnd, dt); late = std::max(late, std::fabs((double)p.heave)); }
        check(early > 1.0, "touchdown produces a bump");
        check(late < early, "touchdown bump decays");
    }

    // Rolling on ground -> rumble present; stationary -> none.
    {
        EffectsLayer e(EffectsConfig::defaults());
        MotionCues roll; roll.onGround = true; roll.groundspeed = 40.0f;
        // skip initial touchdown-edge transient by pre-grounding
        MotionCues gndStill; gndStill.onGround = true; gndStill.groundspeed = 0.0f;
        for (int i=0;i<200;i++) e.update(gndStill, dt);      // let any bump decay
        double rMax=0.0; for (int i=0;i<120;i++){ Pose p=e.update(roll,dt); rMax=std::max(rMax,std::fabs((double)p.heave)); }
        double sMax=0.0; for (int i=0;i<120;i++){ Pose p=e.update(gndStill,dt); sMax=std::max(sMax,std::fabs((double)p.heave)); }
        check(rMax > 0.5, "rolling -> rumble");
        check(sMax < 1e-6, "stationary -> no rumble");
    }

    std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
