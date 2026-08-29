// Offline replay of the motion cueing chain.
//
// Reads a telemetry CSV (only its dt and cue columns matter), runs the same
// WashoutFilter/EffectsLayer/StewartKinematics/SafetyLimiter the plugin runs,
// and writes a telemetry CSV in the identical format. Because the chain is a
// pure function of (cues, dt, config), a replay reproduces exactly what the
// plugin would have computed -- see --verify.
#include "EffectsLayer.h"
#include "MotionConfig.h"
#include "SafetyLimiter.h"
#include "StewartKinematics.h"
#include "Telemetry.h"
#include "WashoutFilter.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <limits>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace {

struct CueSample {
    double     dt = 1.0 / 60.0;
    MotionCues cues;
    // Recorded outputs, kept for --verify.
    float recLiveHeave = 0.0f, recLiveRoll = 0.0f, recLivePitch = 0.0f, recLiveYaw = 0.0f;
    bool  haveRecorded = false;
};

std::vector<std::string> splitLine(const std::string& s) {
    std::vector<std::string> out;
    std::string field;
    std::stringstream ss(s);
    while (std::getline(ss, field, ',')) out.push_back(field);
    return out;
}

// Column name -> index, from the header row.
std::map<std::string, size_t> headerIndex(const std::string& headerLine) {
    std::map<std::string, size_t> idx;
    const std::vector<std::string> h = splitLine(headerLine);
    for (size_t i = 0; i < h.size(); ++i) idx[h[i]] = i;
    return idx;
}

double col(const std::vector<std::string>& f, const std::map<std::string, size_t>& idx,
           const char* name, double fallback = 0.0) {
    const auto it = idx.find(name);
    if (it == idx.end() || it->second >= f.size()) return fallback;
    return std::atof(f[it->second].c_str());
}

bool loadCues(const std::string& path, std::vector<CueSample>& out, std::string& err) {
    std::ifstream in(path);
    if (!in.is_open()) { err = "cannot open " + path; return false; }
    std::string headerLine;
    if (!std::getline(in, headerLine)) { err = "empty file " + path; return false; }
    const std::map<std::string, size_t> idx = headerIndex(headerLine);
    if (idx.find("g_nrml") == idx.end() || idx.find("dt_real") == idx.end()) {
        err = "missing g_nrml/dt_real columns in " + path;
        return false;
    }
    const bool haveRec = idx.find("live_heave") != idx.end();

    const size_t numCols = splitLine(headerLine).size();
    bool warnedRagged = false;
    size_t rowNum = 0;

    std::string line;
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        ++rowNum;
        const std::vector<std::string> f = splitLine(line);
        if (!warnedRagged && f.size() < numCols) {
            std::fprintf(stderr,
                "warning: ragged row %zu has %zu fields, header has %zu -- "
                "recording may be truncated (further ragged rows not reported)\n",
                rowNum, f.size(), numCols);
            warnedRagged = true;
        }
        CueSample s;
        s.dt                = col(f, idx, "dt_real", 1.0 / 60.0);
        s.cues.heaveG       = static_cast<float>(col(f, idx, "g_nrml", 1.0));
        s.cues.surgeG       = static_cast<float>(col(f, idx, "g_axil"));
        s.cues.swayG        = static_cast<float>(col(f, idx, "g_side"));
        s.cues.rollRate     = static_cast<float>(col(f, idx, "P"));
        s.cues.pitchRate    = static_cast<float>(col(f, idx, "Q"));
        s.cues.yawRate      = static_cast<float>(col(f, idx, "R"));
        s.cues.pitchDeg     = static_cast<float>(col(f, idx, "theta"));
        s.cues.rollDeg      = static_cast<float>(col(f, idx, "phi"));
        s.cues.onGround     = col(f, idx, "onground") != 0.0;
        s.cues.groundspeed  = static_cast<float>(col(f, idx, "gs"));
        s.cues.engineRpm    = static_cast<float>(col(f, idx, "rpm"));
        s.cues.alphaDeg     = static_cast<float>(col(f, idx, "alpha"));
        s.cues.simPaused    = col(f, idx, "paused") != 0.0;
        if (haveRec) {
            s.recLiveHeave = static_cast<float>(col(f, idx, "live_heave"));
            s.recLiveRoll  = static_cast<float>(col(f, idx, "live_roll"));
            s.recLivePitch = static_cast<float>(col(f, idx, "live_pitch"));
            s.recLiveYaw   = static_cast<float>(col(f, idx, "live_yaw"));
            s.haveRecorded = true;
        }
        out.push_back(s);
    }
    return true;
}

// --set / --sweep address config fields by "section.key". Pointer-to-member
// tables keep the mapping explicit and complete rather than reflective.
struct WashoutKey { const char* key; double WashoutConfig::* field; };
struct SafetyKey  { const char* key; double SafetyConfig::*  field; };
struct EffectsKey { const char* key; double EffectsConfig::* field; };

const WashoutKey kWashoutKeys[] = {
    {"washout.heave_gain",              &WashoutConfig::heaveGain},
    {"washout.heave_hp_tau",            &WashoutConfig::heaveHpTau},
    {"washout.heave_vel_washout_tau",   &WashoutConfig::heaveVelWashoutTau},
    {"washout.heave_pos_washout_tau",   &WashoutConfig::heavePosWashoutTau},
    {"washout.heave_limit_mm",          &WashoutConfig::heaveLimitMm},
    {"washout.tilt_surge_gain",         &WashoutConfig::tiltSurgeGain},
    {"washout.tilt_sway_gain",          &WashoutConfig::tiltSwayGain},
    {"washout.tilt_lp_tau",             &WashoutConfig::tiltLpTau},
    {"washout.tilt_limit_deg",          &WashoutConfig::tiltLimitDeg},
    {"washout.tilt_rate_limit_dps",     &WashoutConfig::tiltRateLimitDps},
    {"washout.rot_roll_gain",           &WashoutConfig::rotRollGain},
    {"washout.rot_pitch_gain",          &WashoutConfig::rotPitchGain},
    {"washout.rot_yaw_gain",            &WashoutConfig::rotYawGain},
    {"washout.rot_hp_tau",              &WashoutConfig::rotHpTau},
    {"washout.rot_washout_tau",         &WashoutConfig::rotWashoutTau},
    {"washout.rot_limit_deg",           &WashoutConfig::rotLimitDeg},
    {"washout.smooth_tau",              &WashoutConfig::smoothTau},
};
const SafetyKey kSafetyKeys[] = {
    {"safety.max_velocity_cps",      &SafetyConfig::maxVelocity},
    {"safety.max_acceleration_cps2", &SafetyConfig::maxAcceleration},
};
const EffectsKey kEffectsKeys[] = {
    {"effects.touchdown_gain",       &EffectsConfig::touchdownGain},
    {"effects.touchdown_freq_hz",    &EffectsConfig::touchdownFreqHz},
    {"effects.touchdown_decay_tau",  &EffectsConfig::touchdownDecayTau},
    {"effects.rumble_gain",          &EffectsConfig::rumbleGain},
    {"effects.rumble_freq_hz",       &EffectsConfig::rumbleFreqHz},
    {"effects.rumble_speed_ref_mps", &EffectsConfig::rumbleSpeedRefMps},
};

bool applyOverride(const std::string& key, double value,
                   WashoutConfig& w, SafetyConfig& s, EffectsConfig& e) {
    for (const WashoutKey& k : kWashoutKeys) if (key == k.key) { w.*(k.field) = value; return true; }
    for (const SafetyKey&  k : kSafetyKeys)  if (key == k.key) { s.*(k.field) = value; return true; }
    for (const EffectsKey& k : kEffectsKeys) if (key == k.key) { e.*(k.field) = value; return true; }
    return false;
}

void usage() {
    std::printf(
        "usage: washout_replay --cues FILE --config FILE [options]\n"
        "  --set section.key=VALUE   override a config value (repeatable)\n"
        "  --out FILE                write the replayed run as a telemetry CSV\n"
        "  --verify                  check the replay reproduces the recording's live_* columns\n"
        "  --sweep section.key=A,B,C run once per value, printing a summary table\n"
        "  --resample-dt SEC         re-run at a fixed timestep instead of the recorded one\n");
}

// One full pass of the cueing chain over `samples`. Shared by the plain replay
// path, --sweep (run once per swept value) and --verify (run once, compared
// against the recording's live_* columns). Pulling this out of main() is what
// lets --sweep re-run the whole chain per value without duplicating the loop.
struct RunResult {
    size_t samples = 0;
    double durationSec = 0.0;
    double maxLiveErr = 0.0;   // max |replayed - recorded| over the live_* columns
    bool   liveErrIsNaN = false;  // a NaN divergence was seen; maxLiveErr is a sentinel (inf), not a magnitude
    double satHeavePct = 0.0;  // % of active (unpaused) ticks with the heave clamp engaged
    double peakHeaveRawMm = 0.0;
};

RunResult runChain(const std::vector<CueSample>& samples,
                   const StewartGeometry& geo, const WashoutConfig& wcfg,
                   const EffectsConfig& ecfg, const SafetyConfig& scfg,
                   double resampleDt, Telemetry* out) {
    WashoutFilter     washout(wcfg);
    EffectsLayer      effects(ecfg);
    StewartKinematics kin(geo);
    SafetyLimiter     safety(scfg);

    // Seed the limiter the way MotionProvider::initialize does — from the PARK
    // pose, not from home. The limiter is stateful, so a different seed diverges
    // the first ticks' sent[] values from what a real recording shows.
    uint16_t seed[6];
    {
        Pose parkTarget;
        parkTarget.heave = static_cast<float>(scfg.parkHeaveMm);
        const SolveResult s = kin.solve(kin.clampToReachable(parkTarget));
        for (int i = 0; i < 6; ++i) seed[i] = s.setpoints[i];
    }
    safety.reset(seed);

    RunResult res;
    double t = 0.0;
    size_t clamped = 0, counted = 0, activeTicks = 0;
    // Both are held across paused ticks, mirroring MotionProvider's
    // lastLivePose_ and lastEffectsPose_ — the plugin holds them rather than
    // zeroing them, and a zero-dip in the middle of a recording would read as a
    // real signal to anyone inspecting the CSV later.
    Pose live;
    Pose e;

    for (const CueSample& s : samples) {
        const double dtRaw = (resampleDt > 0.0) ? resampleDt : s.dt;
        double dt = dtRaw;
        if (dt > scfg.maxDtSec) dt = scfg.maxDtSec;

        // The plugin gates ONLY the filter update on pause: the IK solve, the
        // limiter and the telemetry write all run every tick against the held
        // pose (MotionProvider::onFlightLoopTick). Replay must mirror that, or a
        // recording containing a pause replays to fewer rows than it has and the
        // limiter state diverges across the gap — which would break the
        // row-for-row --verify below.
        if (!s.cues.simPaused) {
            const Pose w = washout.update(s.cues, dt);
            e = effects.update(s.cues, dt);
            live.heave = w.heave + e.heave;  live.roll  = w.roll  + e.roll;
            live.pitch = w.pitch + e.pitch;  live.yaw   = w.yaw   + e.yaw;
        }

        double scale = 1.0;
        const Pose cmd = kin.clampToReachable(live, &scale);
        const SolveResult sol = kin.solve(cmd);
        uint16_t target[6], sent[6];
        for (int i = 0; i < 6; ++i) target[i] = sol.setpoints[i];
        safety.limit(target, dt, sent);

        const WashoutTrace& tr = washout.trace();
        ++counted;
        // washout.update() didn't run this tick when paused, so tr.heaveClamped
        // still holds whatever the last real tick left behind -- it must not
        // count toward the saturation statistic (the row is still written every
        // tick either way, for row-count parity with the plugin).
        if (!s.cues.simPaused) {
            ++activeTicks;
            if (tr.heaveClamped) ++clamped;
        }
        const double raw = std::fabs(tr.heavePosRaw);
        if (raw > res.peakHeaveRawMm) res.peakHeaveRawMm = raw;

        if (s.haveRecorded) {
            const double d[4] = {
                std::fabs(static_cast<double>(live.heave) - s.recLiveHeave),
                std::fabs(static_cast<double>(live.roll)  - s.recLiveRoll),
                std::fabs(static_cast<double>(live.pitch) - s.recLivePitch),
                std::fabs(static_cast<double>(live.yaw)   - s.recLiveYaw)};
            for (double v : d) {
                // fabs(NaN) is NaN, and "NaN > maxLiveErr" is false by IEEE-754,
                // so a naive max-tracking comparison silently drops a NaN
                // divergence instead of ever seeing it. Latch a sentinel instead.
                if (std::isnan(v)) {
                    res.maxLiveErr   = std::numeric_limits<double>::infinity();
                    res.liveErrIsNaN = true;
                } else if (v > res.maxLiveErr) {
                    res.maxLiveErr = v;
                }
            }
        }

        if (out && out->recording()) {
            TelemetryRow r;
            r.t = t; r.dtReal = dtRaw; r.dtClamped = dt;
            r.cues = s.cues; r.trace = tr;
            r.effects = e; r.live = live; r.commanded = cmd;
            r.reachScale = scale;
            for (int i = 0; i < 6; ++i) { r.setpoints[i] = target[i]; r.sent[i] = sent[i]; }
            r.velClips = safety.velClipCount();
            r.accClips = safety.accClipCount();
            r.armState = 2;
            out->write(r);
        }
        t += dtRaw;
    }
    res.samples     = counted;
    res.durationSec = t;
    res.satHeavePct = activeTicks ? 100.0 * static_cast<double>(clamped) / static_cast<double>(activeTicks) : 0.0;
    return res;
}

}  // namespace

int main(int argc, char** argv) {
    std::string cuesPath, configPath, outPath;
    std::vector<std::pair<std::string, double>> overrides;
    bool                doVerify = false;
    double              resampleDt = 0.0;
    std::string         sweepKey;
    std::vector<double> sweepValues;

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto next = [&](const char* what) -> std::string {
            if (i + 1 >= argc) { std::fprintf(stderr, "%s needs a value\n", what); std::exit(2); }
            return argv[++i];
        };
        if      (a == "--cues")   cuesPath   = next("--cues");
        else if (a == "--config") configPath = next("--config");
        else if (a == "--out")    outPath    = next("--out");
        else if (a == "--set") {
            const std::string kv = next("--set");
            const size_t eq = kv.find('=');
            if (eq == std::string::npos) { std::fprintf(stderr, "--set wants key=value\n"); return 2; }
            overrides.emplace_back(kv.substr(0, eq), std::atof(kv.c_str() + eq + 1));
        }
        else if (a == "--verify")      doVerify = true;
        else if (a == "--resample-dt") resampleDt = std::atof(next("--resample-dt").c_str());
        else if (a == "--sweep") {
            const std::string kv = next("--sweep");
            const size_t eq = kv.find('=');
            if (eq == std::string::npos) { std::fprintf(stderr, "--sweep wants key=v1,v2\n"); return 2; }
            sweepKey = kv.substr(0, eq);
            std::stringstream ss(kv.substr(eq + 1));
            std::string v;
            while (std::getline(ss, v, ',')) sweepValues.push_back(std::atof(v.c_str()));
        } else { usage(); return 2; }
    }
    if (cuesPath.empty() || configPath.empty()) { usage(); return 2; }

    std::vector<CueSample> samples;
    std::string err;
    if (!loadCues(cuesPath, samples, err)) { std::fprintf(stderr, "%s\n", err.c_str()); return 1; }

    StewartGeometry geo = MotionConfig::loadGeometry(configPath);
    WashoutConfig   wcfg = MotionConfig::loadWashout(configPath);
    EffectsConfig   ecfg = MotionConfig::loadEffects(configPath);
    SafetyConfig    scfg = MotionConfig::loadSafety(configPath);

    for (const auto& o : overrides) {
        if (!applyOverride(o.first, o.second, wcfg, scfg, ecfg)) {
            std::fprintf(stderr, "unknown key: %s\n", o.first.c_str());
            return 2;
        }
    }

    if (!sweepKey.empty()) {
        if (doVerify) {
            std::fprintf(stderr, "VERIFY REFUSED: --verify does not run with --sweep\n");
            return 2;
        }
        // Validate the key once, before anything goes to stdout -- otherwise an
        // unknown key still fails (correctly, exit 2) but only after the table
        // header has already been printed.
        {
            WashoutConfig w = wcfg; SafetyConfig s = scfg; EffectsConfig e = ecfg;
            if (!applyOverride(sweepKey, 0.0, w, s, e)) {
                std::fprintf(stderr, "unknown key: %s\n", sweepKey.c_str());
                return 2;
            }
        }
        std::printf("%-28s %10s %10s %14s\n", sweepKey.c_str(),
                    "sat_heave%", "peak_raw_mm", "samples");
        for (double v : sweepValues) {
            WashoutConfig w = wcfg; SafetyConfig s = scfg; EffectsConfig e = ecfg;
            applyOverride(sweepKey, v, w, s, e);  // key already validated above
            Telemetry sweepOut;
            if (!outPath.empty()) {
                char name[512];
                std::snprintf(name, sizeof(name), "%s.%g.csv", outPath.c_str(), v);
                sweepOut.start(name);
            }
            const RunResult r = runChain(samples, geo, w, e, s, resampleDt,
                                         outPath.empty() ? nullptr : &sweepOut);
            sweepOut.stop();
            std::printf("%-28g %10.2f %10.1f %14zu\n", v, r.satHeavePct,
                        r.peakHeaveRawMm, r.samples);
        }
        return 0;
    }

    Telemetry out;
    if (!outPath.empty() && !out.start(outPath)) {
        std::fprintf(stderr, "cannot write %s\n", outPath.c_str());
        return 1;
    }
    const RunResult r = runChain(samples, geo, wcfg, ecfg, scfg, resampleDt,
                                 outPath.empty() ? nullptr : &out);
    out.stop();
    std::printf("replayed %zu samples (%.1f s), heave saturated %.2f%% of ticks, "
                "peak raw heave %.1f mm\n",
                r.samples, r.durationSec, r.satHeavePct, r.peakHeaveRawMm);

    if (doVerify) {
        if (overrides.empty() && resampleDt == 0.0) {
            // A --synth (or otherwise bare) cue stream carries no recorded
            // live_* columns at all, so maxLiveErr never accumulates and would
            // stay 0.0 -- a false PASS on the one gate the campaign treats as
            // blocking. Refuse instead of reporting success on nothing.
            const bool anyRecorded = std::any_of(samples.begin(), samples.end(),
                [](const CueSample& s) { return s.haveRecorded; });
            if (!anyRecorded) {
                std::fprintf(stderr,
                    "VERIFY FAILED: no sample carried recorded live_* columns -- "
                    "nothing to verify against (synthetic or bare cue stream?)\n");
                return 1;
            }
            std::printf("verify: max |replay - recorded| over live_* = %.9g mm/deg\n", r.maxLiveErr);
            if (r.liveErrIsNaN) {
                std::fprintf(stderr,
                    "VERIFY FAILED: a NaN divergence was encountered in the live_* "
                    "comparison (the max above is an inf sentinel, not a magnitude)\n");
                return 1;
            }
            if (r.maxLiveErr != 0.0) {
                std::fprintf(stderr, "VERIFY FAILED: replay does not reproduce the recording\n");
                return 1;
            }
            std::printf("verify: PASS (bit-exact)\n");
        } else {
            std::fprintf(stderr,
                "VERIFY REFUSED: --verify requires no --set and no --resample-dt\n");
            return 2;
        }
    }
    return 0;
}
