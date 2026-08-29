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

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
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
        "  --out FILE                write the replayed run as a telemetry CSV\n");
}

}  // namespace

int main(int argc, char** argv) {
    std::string cuesPath, configPath, outPath;
    std::vector<std::pair<std::string, double>> overrides;

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

    WashoutFilter      washout(wcfg);
    EffectsLayer       effects(ecfg);
    StewartKinematics  kin(geo);
    SafetyLimiter      safety(scfg);

    // Replay is always "armed and live": the arm blend is a transition, not part
    // of the cueing chain under test. Still seed the limiter from the park pose
    // (as MotionProvider::initialize does), not home -- the limiter is stateful,
    // so a different seed diverges the first ticks' sent[]/clip counts from a
    // real recording.
    Pose park;
    park.heave = static_cast<float>(scfg.parkHeaveMm);
    const Pose parkClamped = kin.clampToReachable(park);
    const SolveResult parkSolve = kin.solve(parkClamped);
    uint16_t parkSetpoints[6];
    for (int i = 0; i < 6; ++i) parkSetpoints[i] = parkSolve.setpoints[i];
    safety.reset(parkSetpoints);

    Telemetry out;
    if (!outPath.empty() && !out.start(outPath)) {
        std::fprintf(stderr, "cannot write %s\n", outPath.c_str());
        return 1;
    }

    // Persists across paused ticks, mirroring MotionProvider::lastLivePose_: the
    // flight loop keeps ticking with wall-clock dt while the sim is paused, but
    // the filters must not see that time, so the last live pose is held instead
    // of integrated.
    Pose live;
    double t = 0.0;
    for (const CueSample& s : samples) {
        double dt = s.dt;
        if (dt > scfg.maxDtSec) dt = scfg.maxDtSec;

        // This tick's effects contribution only, for the telemetry row -- zero
        // on a paused tick (the held `live` pose above already carries the
        // combined value forward, matching clampToReachable/solve/limit/write
        // running unconditionally every tick just like MotionProvider does).
        Pose e;
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

        if (out.recording()) {
            TelemetryRow r;
            r.t = t; r.dtReal = s.dt; r.dtClamped = dt;
            r.cues = s.cues; r.trace = washout.trace();
            r.effects = e; r.live = live; r.commanded = cmd;
            r.reachScale = scale;
            for (int i = 0; i < 6; ++i) { r.setpoints[i] = target[i]; r.sent[i] = sent[i]; }
            r.velClips = safety.velClipCount();
            r.accClips = safety.accClipCount();
            r.armState = 2;   // Armed
            out.write(r);
        }
        t += s.dt;
    }
    out.stop();
    std::printf("replayed %zu samples (%.1f s)\n", samples.size(), t);
    return 0;
}
