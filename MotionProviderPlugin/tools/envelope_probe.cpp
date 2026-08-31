// Measures the platform's reachable surge/sway travel, bare and in the corner
// of the envelope the other channels already occupy. Links the real
// StewartKinematics -- there is deliberately no second implementation of the
// geometry that could drift from the plugin's.
#include "MotionConfig.h"
#include "StewartKinematics.h"

#include <cstdio>
#include <cstring>
#include <string>

namespace {

bool reachable(const StewartKinematics& k, const Pose& p) {
    return k.solve(p).allReachable;
}

// Largest travel along `axis` in direction `dir` that still solves, searched in
// [0, hi] by bisection. Assumes `base` is reachable and that reachability is
// monotone along the axis -- true here: the legs run out of travel as the pose
// moves away from home, they do not come back.
double maxTravel(const StewartKinematics& k, const Pose& base,
                 float Pose::*axis, double dir, double hi) {
    Pose p = base;
    p.*axis = static_cast<float>(base.*axis + dir * hi);
    if (reachable(k, p)) return hi;      // never ran out inside the search range
    double lo = 0.0;
    for (int i = 0; i < 60; ++i) {
        const double mid = 0.5 * (lo + hi);
        p = base;
        p.*axis = static_cast<float>(base.*axis + dir * mid);
        if (reachable(k, p)) lo = mid; else hi = mid;
    }
    return lo;
}

void report(const StewartKinematics& k, const char* label, const Pose& base) {
    if (!reachable(k, base)) {
        std::printf("%-28s BASE POSE UNREACHABLE\n", label);
        return;
    }
    const double sPos = maxTravel(k, base, &Pose::surge, +1.0, 500.0);
    const double sNeg = maxTravel(k, base, &Pose::surge, -1.0, 500.0);
    const double yPos = maxTravel(k, base, &Pose::sway,  +1.0, 500.0);
    const double yNeg = maxTravel(k, base, &Pose::sway,  -1.0, 500.0);
    std::printf("%-28s surge +%7.2f / -%7.2f    sway +%7.2f / -%7.2f  (mm)\n",
                label, sPos, sNeg, yPos, yNeg);
}

}  // namespace

int main(int argc, char** argv) {
    std::string cfgPath = "configuration.toml";
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--config") == 0 && i + 1 < argc) cfgPath = argv[++i];
    }

    bool loaded = false;
    const StewartGeometry geo = MotionConfig::loadGeometry(cfgPath, &loaded);
    std::printf("config: %s (%s)\n", cfgPath.c_str(), loaded ? "loaded" : "DEFAULTS -- file not read");
    StewartKinematics kin(geo);
    std::printf("home height: %.2f mm\n\n", kin.homeHeight());

    // Bare: an upper bound, never available in flight.
    report(kin, "home pose", Pose{});

    // In the corner: heave at its limit and tilt+rotational at their combined
    // per-axis limit, i.e. what the existing channels are already allowed to
    // occupy at the same time. This is the number the per-axis limits come from.
    for (double hs : {+1.0, -1.0}) {
        for (double as : {+1.0, -1.0}) {
            Pose c;
            c.heave = static_cast<float>(hs * 30.0);
            c.roll  = static_cast<float>(as * 14.0);
            c.pitch = static_cast<float>(as * 14.0);
            c.yaw   = static_cast<float>(as * 7.0);
            char label[64];
            std::snprintf(label, sizeof(label), "corner h%+.0f r/p%+.0f y%+.0f",
                          c.heave, c.roll, c.yaw);
            report(kin, label, c);
        }
    }
    return 0;
}
