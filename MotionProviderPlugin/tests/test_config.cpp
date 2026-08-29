#include "MotionConfig.h"
#include "StewartGeometry.h"
#include "WashoutConfig.h"
#include "EffectsConfig.h"
#include "TelemetryConfig.h"
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>
#include <cmath>

static int g_failures = 0, g_checks = 0;
static void check(bool c, const char* w){ ++g_checks; if(!c){++g_failures; std::printf("  FAIL: %s\n", w);} }
static void near(double a,double b,const char*w){ ++g_checks; if(std::fabs(a-b)>1e-9){++g_failures; std::printf("  FAIL: %s (%.6f vs %.6f)\n",w,a,b);} }

static std::string tmpPath() {
    return std::string(std::getenv("TMPDIR") ? std::getenv("TMPDIR") : "/tmp") + "/mp_test.toml";
}

int main() {
    // Missing file -> defaults, outLoaded=false.
    {
        bool loaded = true;
        StewartGeometry g = MotionConfig::loadGeometry("/no/such/file.toml", &loaded);
        StewartGeometry d = StewartGeometry::defaults();
        check(!loaded, "missing file -> outLoaded false");
        near(g.baseRadius, d.baseRadius, "missing file -> default Rb");
        near(g.rodLength,  d.rodLength,  "missing file -> default rod");
    }

    // Partial file with FLOAT literals -> overrides present keys, defaults the rest.
    {
        std::string tmp = tmpPath();
        { std::ofstream f(tmp);
          f << "[geometry]\n"
               "base_radius_mm = 500.0\n"
               "base_angle_deg = [1.0,2.0,3.0,4.0,5.0,6.0]\n"
               "[servo]\n"
               "demand_home = 30000\n"; }
        bool loaded = false;
        StewartGeometry g = MotionConfig::loadGeometry(tmp, &loaded);
        StewartGeometry d = StewartGeometry::defaults();
        check(loaded, "present file -> outLoaded true");
        near(g.baseRadius, 500.0, "override Rb (float)");
        near(g.platformRadius, d.platformRadius, "default Rp kept");
        near(g.phiDeg[0], 1.0, "override phi[0] (float array)");
        near(g.phiDeg[5], 6.0, "override phi[5] (float array)");
        check(g.demandHome == 30000, "override demand_home");
        check(g.demandMax == d.demandMax, "default demand_max kept");
        std::remove(tmp.c_str());
    }

    // Regression: INTEGER literals must be honored for double fields too
    // (toml++ value<double>() rejects ints; the loader must fall back to int64).
    {
        std::string tmp = tmpPath();
        { std::ofstream f(tmp);
          f << "[geometry]\n"
               "base_radius_mm = 500\n"                 // int for a double field
               "horn_length_mm = 120\n"                 // the exact case that failed
               "base_angle_deg = [1,2,3,4,5,6]\n"       // int array for double field
               "[servo]\n"
               "angle_at_full_scale_deg = 60\n"; }      // int for a double field
        StewartGeometry g = MotionConfig::loadGeometry(tmp);
        near(g.baseRadius, 500.0, "int scalar honored (Rb)");
        near(g.hornLength, 120.0, "int scalar honored (horn)");
        near(g.phiDeg[0], 1.0, "int array honored phi[0]");
        near(g.phiDeg[5], 6.0, "int array honored phi[5]");
        near(g.angleAtFullScale, 60.0, "int scalar honored (angle_at_full_scale)");
        std::remove(tmp.c_str());
    }

    // [washout] / [effects] partial override + defaults.
    {
        std::string tmp = tmpPath();
        { std::ofstream f(tmp);
          f << "[washout]\n"
               "heave_gain = 0.9\n"
               "tilt_limit_deg = 4\n"          // int for a double field
               "[effects]\n"
               "rumble_gain = 3.5\n"; }
        WashoutConfig w = MotionConfig::loadWashout(tmp);
        EffectsConfig e = MotionConfig::loadEffects(tmp);
        WashoutConfig wd = WashoutConfig::defaults();
        EffectsConfig ed = EffectsConfig::defaults();
        near(w.heaveGain, 0.9, "washout heave_gain override");
        near(w.tiltLimitDeg, 4.0, "washout tilt_limit_deg int override");
        near(w.rotRollGain, wd.rotRollGain, "washout default kept");
        near(e.rumbleGain, 3.5, "effects rumble_gain override");
        near(e.touchdownGain, ed.touchdownGain, "effects default kept");
        std::remove(tmp.c_str());
    }

    // writeDefaults produces a file that round-trips back to the defaults.
    {
        std::string tmp = tmpPath();
        check(MotionConfig::writeDefaults(tmp), "writeDefaults succeeds");
        bool loaded = false;
        StewartGeometry g = MotionConfig::loadGeometry(tmp, &loaded);
        WashoutConfig   w = MotionConfig::loadWashout(tmp);
        SafetyConfig    s = MotionConfig::loadSafety(tmp);
        StewartGeometry gd = StewartGeometry::defaults();
        WashoutConfig   wd = WashoutConfig::defaults();
        SafetyConfig    sd = SafetyConfig::defaults();
        check(loaded, "written file parses");
        near(g.baseRadius, gd.baseRadius, "roundtrip Rb");
        near(g.psiDeg[1], gd.psiDeg[1], "roundtrip anchor angle (float)");
        near(g.phiDeg[0], gd.phiDeg[0], "roundtrip base angle (int-looking)");
        near(w.rotWashoutTau, wd.rotWashoutTau, "roundtrip washout tau");
        near(s.parkHeaveMm, sd.parkHeaveMm, "roundtrip park heave");
        near(s.armRampSec, sd.armRampSec, "roundtrip arm ramp");
        std::remove(tmp.c_str());
    }

    // [telemetry] loads, and absent keys fall back to defaults.
    {
        const std::string p = "test_config_telemetry.toml";
        {
            std::ofstream f(p);
            f << "[telemetry]\n";
            f << "enabled = true\n";
            f << "dir = \"/tmp/motion\"\n";
        }
        TelemetryConfig t = MotionConfig::loadTelemetry(p);
        check(t.enabled, "telemetry.enabled parsed");
        check(t.dir == "/tmp/motion", "telemetry.dir parsed");
        std::remove(p.c_str());
    }
    {
        TelemetryConfig t = MotionConfig::loadTelemetry("does_not_exist.toml");
        check(!t.enabled && t.dir.empty(), "missing file -> telemetry defaults");
    }

    std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
