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
        check(hc > 40, "header has the full column set");
        check(headerLine.rfind("t_sec,", 0) == 0, "header starts with t_sec");
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
