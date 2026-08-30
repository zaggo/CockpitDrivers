#include "Telemetry.h"
#include <clocale>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

static int g_failures = 0, g_checks = 0;
static void check(bool c, const char* w){ ++g_checks; if(!c){++g_failures; std::printf("  FAIL: %s\n", w);} }

static std::vector<std::string> split(const std::string& s) {
    std::vector<std::string> out;
    std::stringstream ss(s);
    std::string f;
    while (std::getline(ss, f, ',')) out.push_back(f);
    return out;
}

int main() {
    const std::string path = "test_telemetry_out.csv";

    // Header and data rows must agree on column count, or every downstream
    // parser silently misreads the file.
    {
        Telemetry t;
        check(t.start(path), "start() opens the file");
        check(t.recording(), "recording() true after start");
        TelemetryRow r;
        r.t = 1.25; r.dtReal = 1.0/60.0; r.dtClamped = 1.0/60.0;
        r.cues.heaveG = 1.5f;
        r.trace.heavePosRaw = 123.456;
        r.trace.heaveClamped = true;
        r.live.heave = 30.0f;
        r.commanded.heave = 12.0f;
        r.reachScale = 0.75;
        for (int i = 0; i < 6; ++i) { r.setpoints[i] = 1000 + i; r.sent[i] = 2000 + i; }
        r.velClips = 3; r.accClips = 1; r.armState = 2;
        r.effState.prevOnGround = true;
        r.effState.tdActive     = true;
        r.effState.tdT          = 0.5;
        r.effState.rumblePhase  = 2.345;
        t.write(r);
        t.write(r);
        check(t.rows() == 2, "rows() counts written rows");
        t.stop();
        check(!t.recording(), "recording() false after stop");
    }
    {
        std::ifstream in(path);
        std::string headerLine, dataLine;
        std::getline(in, headerLine);
        std::getline(in, dataLine);
        const size_t hc = split(headerLine).size();
        const size_t dc = split(dataLine).size();
        check(hc == dc, "header and data column counts agree");
        // Exact, not a loose lower bound: this is the one place the schema's
        // full column count is asserted. 57 columns originally, +4 for the
        // effects state (eff_prev_onground, eff_td_active, eff_td_t,
        // eff_rumble_phase), +5 for the slab-joint state (eff_slab_dist,
        // eff_slab_from, eff_slab_to, eff_slab_t, eff_slab_dur) = 66. A bound
        // loose enough to survive an addition silently would also survive one
        // of them going missing -- and a missing state column costs a replay
        // its bit-exactness without any other symptom.
        check(hc == 66, "header has exactly the documented 66-column schema");
        check(headerLine.rfind("t_sec,", 0) == 0, "header starts with t_sec");
        check(headerLine.find(",eff_prev_onground,eff_td_active,eff_td_t,eff_rumble_phase")
                  != std::string::npos,
              "the Change-2 effects-state columns are present, in order, at the end");
        check(headerLine.find(",eff_slab_dist,eff_slab_from,eff_slab_to,eff_slab_t,eff_slab_dur")
                  != std::string::npos,
              "the slab-joint state columns are present, in order, at the end");
    }

    // Change 2's effects-state columns round-trip exactly, booleans as 0/1
    // and doubles at full precision -- washout_replay's seeding depends on
    // reading back exactly what was recorded, particularly rumblePhase
    // (unlike a filter's decaying state, a phase mismatch never washes out).
    {
        Telemetry t;
        t.start(path);
        TelemetryRow r;
        r.effState.prevOnGround = true;
        r.effState.tdActive     = false;
        r.effState.tdT          = 1.0 / 3.0;
        r.effState.rumblePhase  = 4.71238898038469;  // 3*pi/2, an awkward double
        t.write(r);
        t.stop();
        std::ifstream in(path);
        std::string headerLine, dataLine;
        std::getline(in, headerLine);
        std::getline(in, dataLine);
        const std::vector<std::string> h = split(headerLine);
        const std::vector<std::string> d = split(dataLine);
        size_t pogIdx = h.size(), tdaIdx = h.size(), tdtIdx = h.size(), phIdx = h.size();
        for (size_t i = 0; i < h.size(); ++i) {
            if (h[i] == "eff_prev_onground") pogIdx = i;
            if (h[i] == "eff_td_active")     tdaIdx = i;
            if (h[i] == "eff_td_t")          tdtIdx = i;
            if (h[i] == "eff_rumble_phase")  phIdx  = i;
        }
        check(pogIdx < h.size() && tdaIdx < h.size() && tdtIdx < h.size() && phIdx < h.size(),
              "all four effects-state columns exist");
        check(pogIdx < d.size() && std::atoi(d[pogIdx].c_str()) == 1,
              "eff_prev_onground round-trips as 1 (true)");
        check(tdaIdx < d.size() && std::atoi(d[tdaIdx].c_str()) == 0,
              "eff_td_active round-trips as 0 (false)");
        check(tdtIdx < d.size() && std::atof(d[tdtIdx].c_str()) == r.effState.tdT,
              "eff_td_t round-trips exactly");
        check(phIdx < d.size() && std::atof(d[phIdx].c_str()) == r.effState.rumblePhase,
              "eff_rumble_phase round-trips exactly");
    }

    // Floats round-trip exactly at the documented precision -- the bit-exact
    // replay self-test depends on this.
    {
        Telemetry t;
        t.start(path);
        TelemetryRow r;
        r.cues.heaveG = 1.0f + 1.0f/3.0f;
        r.dtReal = 1.0/60.0;
        t.write(r);
        t.stop();
        std::ifstream in(path);
        std::string headerLine, dataLine;
        std::getline(in, headerLine);
        std::getline(in, dataLine);
        const std::vector<std::string> h = split(headerLine);
        const std::vector<std::string> d = split(dataLine);
        size_t gIdx = h.size(), dtIdx = h.size();
        for (size_t i = 0; i < h.size(); ++i) {
            if (h[i] == "g_nrml")  gIdx = i;
            if (h[i] == "dt_real") dtIdx = i;
        }
        check(gIdx < h.size() && dtIdx < h.size(), "g_nrml and dt_real columns exist");
        check(static_cast<float>(std::atof(d[gIdx].c_str())) == r.cues.heaveG,
              "float column round-trips exactly");
        check(std::atof(d[dtIdx].c_str()) == r.dtReal,
              "double column round-trips exactly");
    }

    // The writer must be locale-INDEPENDENT. This plugin is a shared library
    // inside X-Plane; the host, or any other loaded plugin, may call
    // setlocale(). Under a comma-decimal locale a printf/stream-based writer
    // emits "0,0166..." and every row silently gains columns -- and a
    // recording is written once and cannot be regenerated. std::to_chars is
    // locale-independent by definition; this test is what holds that.
    {
        const char* prev = std::setlocale(LC_ALL, nullptr);
        const std::string saved = prev ? prev : "C";
        // Any comma-decimal locale will do; skip if none is installed.
        const char* got = std::setlocale(LC_ALL, "de_DE.UTF-8");
        if (!got) got = std::setlocale(LC_ALL, "de_DE");
        if (!got) got = std::setlocale(LC_ALL, "fr_FR.UTF-8");
        if (!got) {
            std::printf("  (skipped: no comma-decimal locale installed)\n");
        } else {
            Telemetry t;
            t.start(path);
            TelemetryRow r;
            r.t = 1.0 / 3.0;
            r.dtReal = 1.0 / 60.0;
            r.cues.heaveG = 1.0f + 1.0f / 3.0f;
            r.reachScale = 0.5;
            t.write(r);
            t.stop();
            // Parse back under the C locale -- that is what every downstream
            // tool (washout_replay, washout_metrics.py) actually uses.
            std::setlocale(LC_ALL, saved.c_str());
            std::ifstream in(path);
            std::string headerLine, dataLine;
            std::getline(in, headerLine);
            std::getline(in, dataLine);
            const std::vector<std::string> h = split(headerLine);
            const std::vector<std::string> d = split(dataLine);
            check(h.size() == d.size(),
                  "comma-decimal locale does not add columns to a data row");
            size_t gIdx = h.size();
            for (size_t i = 0; i < h.size(); ++i) if (h[i] == "g_nrml") gIdx = i;
            check(gIdx < d.size() &&
                      static_cast<float>(std::atof(d[gIdx].c_str())) == r.cues.heaveG,
                  "float written under a comma-decimal locale still round-trips");
            check(!d.empty() && std::atof(d[0].c_str()) == r.t,
                  "t_sec written under a comma-decimal locale still round-trips");
        }
        std::setlocale(LC_ALL, saved.c_str());
    }

    std::remove(path.c_str());
    std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
