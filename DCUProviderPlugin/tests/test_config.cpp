#include "ConfigUtils.h"
#include "AxisCurve.h"
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>
#include <unistd.h>

static int g_failures = 0, g_checks = 0;
static void check(bool c, const char* w) { ++g_checks; if (!c) { ++g_failures; std::printf("  FAIL: %s\n", w); } }

static void writeConfig(const std::string& body) {
    std::ofstream f(getConfigFilePath(), std::ios::trunc);
    f << body;
}

int main() {
    // ConfigUtils resolves the file from $HOME on every call, so point HOME at
    // a fresh temp dir. Honour TMPDIR so sandboxed runs can write it.
    const char* tmp = std::getenv("TMPDIR");
    std::string tmpl = std::string(tmp ? tmp : "/tmp") + "/dcucfgXXXXXX";
    const char* dir = mkdtemp(&tmpl[0]);
    if (!dir) { std::printf("mkdtemp failed\n"); return 1; }
    setenv("HOME", dir, 1);

    // No file at all -> defaults.
    {
        std::remove(getConfigFilePath().c_str());
        AxisCurve c = loadAxisCurve("rudder");
        check(c.sensPos == 0.f && c.sensNeg == 0.f && c.neutral == 0.f && c.deadZone == 0.f && c.extremityDeadZone == 0.f,
              "missing file -> all zero");
        check(loadCurveWindowVisible() == false, "missing file -> curve window hidden");
    }

    // Round trip.
    {
        AxisCurve c; c.sensPos = -0.35f; c.sensNeg = 0.25f; c.neutral = 0.1f; c.deadZone = 0.05f; c.extremityDeadZone = 0.2f;
        saveAxisCurve("rudder", c);
        AxisCurve r = loadAxisCurve("rudder");
        check(r.sensPos == -0.35f && r.sensNeg == 0.25f, "round trip: sensitivities");
        check(r.neutral == 0.1f && r.deadZone == 0.05f && r.extremityDeadZone == 0.2f, "round trip: neutral/dz/ext");
    }

    // Saving the curve keeps unrelated keys.
    {
        saveLastUsedPort("/dev/tty.fake");
        AxisCurve c; c.sensPos = 0.5f;
        saveAxisCurve("rudder", c);
        check(loadLastUsedPort() == "/dev/tty.fake", "saveAxisCurve keeps other keys");
        check(loadAxisCurve("rudder").sensPos == 0.5f, "...and writes its own");
    }

    // Axis prefix isolates axes.
    {
        AxisCurve a; a.neutral = 0.3f;
        saveAxisCurve("left_brake", a);
        check(loadAxisCurve("rudder").neutral == 0.f, "left_brake keys do not touch rudder");
        check(loadAxisCurve("left_brake").neutral == 0.3f, "left_brake loads its own value");
    }

    // Partial file: present keys kept, missing ones default (Review Focus 5).
    {
        writeConfig("curve.rudder.sens_pos=-0.5\ncurve.rudder.dead_zone=0.1\n");
        AxisCurve c = loadAxisCurve("rudder");
        check(c.sensPos == -0.5f && c.deadZone == 0.1f, "partial: present keys loaded");
        check(c.sensNeg == 0.f && c.neutral == 0.f && c.extremityDeadZone == 0.f, "partial: missing keys default to 0");
    }

    // Garbage values: no exception, clamped or zero (Review Focus 2).
    {
        writeConfig("curve.rudder.sens_pos=abc\ncurve.rudder.sens_neg=nan\ncurve.rudder.neutral=5\ncurve.rudder.dead_zone=-3\ncurve.rudder.extremity=\n");
        AxisCurve c = loadAxisCurve("rudder");
        check(c.sensPos == 0.f, "garbage 'abc' -> 0");
        check(c.sensNeg == 0.f, "garbage 'nan' -> 0");
        check(c.neutral == 1.f, "out of range 5 -> clamped to 1");
        check(c.deadZone == 0.f, "out of range -3 -> clamped to 0");
        check(c.extremityDeadZone == 0.f, "empty value -> 0");
    }

    // Window visibility round trip.
    {
        saveCurveWindowVisible(true);
        check(loadCurveWindowVisible() == true, "curve window visible round trip (true)");
        saveCurveWindowVisible(false);
        check(loadCurveWindowVisible() == false, "curve window visible round trip (false)");
    }

    std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
