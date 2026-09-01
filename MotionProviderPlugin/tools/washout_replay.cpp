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

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// --verify's pass tolerance, in the same mm/deg units as maxLiveErr.
//
// Not 0.0. Skipping the warm-up window (see verifyWarmupSec below) removes
// the DOMINANT source of divergence -- an initial condition the recording
// never captured -- but that source decays asymptotically, never landing on
// literal IEEE bit-for-bit equality within a finite recording: measured on
// the campaign's real recordings, the rotational channels (whose washout
// carries the config's slowest time constant, rot_washout_tau) still show
// isolated ~1e-12 mm/deg blips deep into the file, well past any reasonable
// warm-up -- residual float32-rounding noise from two independently
// converging trajectories, not a reproduction defect. Requiring exact
// equality would make --verify FAIL on every real recording forever, which
// defeats the point of this fix. 1e-4 mm/deg is the noise ceiling actually
// observed (see docs/motion-tuning/README.md's --verify section) -- four
// orders of magnitude below the 0.415 mm divergence a genuine reproduction
// failure showed before this fix, so it stays a meaningful gate.
constexpr double kVerifyToleranceMmDeg = 1e-4;

struct CueSample {
    double     dt = 1.0 / 60.0;
    MotionCues cues;
    // Recorded outputs, kept for --verify.
    float recLiveHeave = 0.0f, recLiveRoll = 0.0f, recLivePitch = 0.0f, recLiveYaw = 0.0f;
    // Recorded by the 78-column schema onward. A recording that predates those
    // columns reads them as 0, which matches what the filter produced then:
    // the channel did not exist, so live surge/sway were 0.
    float recLiveSurge = 0.0f, recLiveSway = 0.0f;
    bool  haveRecorded = false;
    // Recorded ArmState (0 Disarmed, 1 Arming, 2 Armed, 3 Disarming). Needed
    // because MotionProvider resets the stateful filters on the rising edge
    // into an armed state -- see runChain. Absent for synthetic streams and
    // for older/cue-only files, which then simply carry no edges.
    int  armState     = 0;
    bool haveArmState = false;
    // EffectsLayer's recorded internal state for this row (Change 2 -- see
    // Telemetry's eff_* state columns). Only row 0's is ever actually used
    // (to seed the layer before processing anything -- see runChain), but
    // every row is parsed the same way for consistency. Absent for every
    // recording that predates these columns, which is every one of this
    // campaign's seven real recordings -- absent means "seed from zero",
    // i.e. today's behaviour, unchanged.
    EffectsLayer::State effState;
    bool haveEffState = false;
};

std::vector<std::string> splitLine(const std::string& s) {
    std::vector<std::string> out;
    std::string field;
    std::stringstream ss(s);
    while (std::getline(ss, field, ',')) out.push_back(field);
    // A recording made on the Sim-PC is CRLF: std::ofstream's text mode turns
    // Telemetry's '\n' into "\r\n" on Windows, and reading that file back on
    // macOS leaves the '\r' glued to the LAST field of every line. On the
    // header row that renames the last column ("arm_state" -> "arm_state\r"),
    // so its lookup silently misses and the column reads as its fallback --
    // which would quietly disable the arm-edge reset modelled in runChain.
    if (!out.empty() && !out.back().empty() && out.back().back() == '\r')
        out.back().pop_back();
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
    const bool haveArm = idx.find("arm_state") != idx.end();
    // Change 2's effects-state columns are optional and travel as a group --
    // partial presence (an edited/hand-built file missing one) is treated the
    // same as none, so seeding never runs off a half-populated state. Every
    // one of this campaign's seven real recordings predates these columns, so
    // this is false for all of them and effState stays at its zero default,
    // which is exactly today's (pre-Change-2) seeding behaviour.
    const bool haveEffState =
        idx.find("eff_prev_onground") != idx.end() && idx.find("eff_td_active") != idx.end() &&
        idx.find("eff_td_t") != idx.end() && idx.find("eff_rumble_phase") != idx.end();
    // The slab-joint state is a second such group, added later still. It is
    // gated separately so a recording from between the two changes seeds the
    // columns it does have instead of silently seeding none -- and a zero slab
    // state is the correct seed for those files, because the effect did not
    // exist when they were made.
    const bool haveSlabState =
        idx.find("eff_slab_dist") != idx.end() && idx.find("eff_slab_from") != idx.end() &&
        idx.find("eff_slab_to") != idx.end() && idx.find("eff_slab_t") != idx.end() &&
        idx.find("eff_slab_dur") != idx.end();

    const size_t numCols = splitLine(headerLine).size();
    bool warnedRagged = false;
    size_t rowNum = 0;

    std::string line;
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        ++rowNum;
        const std::vector<std::string> f = splitLine(line);
        // Both directions matter. A SHORT row means a truncated recording; a
        // LONG row means every col() lookup past the extra field reads by
        // header index into shifted data, which is worse (it produces
        // plausible numbers from the wrong columns) and used to pass in
        // silence.
        if (!warnedRagged && f.size() != numCols) {
            std::fprintf(stderr,
                "warning: ragged row %zu has %zu fields, header has %zu -- "
                "columns may be misaligned or the recording truncated "
                "(further ragged rows not reported)\n",
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
            s.recLiveSurge = static_cast<float>(col(f, idx, "live_surge", 0.0));
            s.recLiveSway  = static_cast<float>(col(f, idx, "live_sway",  0.0));
            s.haveRecorded = true;
        }
        if (haveArm) {
            s.armState     = static_cast<int>(col(f, idx, "arm_state"));
            s.haveArmState = true;
        }
        if (haveEffState) {
            s.effState.prevOnGround = col(f, idx, "eff_prev_onground") != 0.0;
            s.effState.tdActive     = col(f, idx, "eff_td_active") != 0.0;
            s.effState.tdT          = col(f, idx, "eff_td_t");
            s.effState.rumblePhase  = col(f, idx, "eff_rumble_phase");
            s.haveEffState          = true;
        }
        if (haveSlabState) {
            s.effState.slabDist = col(f, idx, "eff_slab_dist");
            s.effState.slabFrom = col(f, idx, "eff_slab_from");
            s.effState.slabTo   = col(f, idx, "eff_slab_to");
            s.effState.slabT    = col(f, idx, "eff_slab_t");
            s.effState.slabDur  = col(f, idx, "eff_slab_dur");
            s.haveEffState      = true;
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
    {"washout.surge_gain",              &WashoutConfig::surgeGain},
    {"washout.sway_gain",               &WashoutConfig::swayGain},
    {"washout.trans_vel_washout_tau",   &WashoutConfig::transVelWashoutTau},
    {"washout.trans_pos_washout_tau",   &WashoutConfig::transPosWashoutTau},
    {"washout.surge_limit_mm",          &WashoutConfig::surgeLimitMm},
    {"washout.sway_limit_mm",           &WashoutConfig::swayLimitMm},
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
    {"effects.slab_spacing_m",       &EffectsConfig::slabSpacingM},
    {"effects.slab_step_mm",         &EffectsConfig::slabStepMm},
    {"effects.slab_accel_mm_s2",     &EffectsConfig::slabAccelMmS2},
    {"effects.slab_min_speed_mps",   &EffectsConfig::slabMinSpeedMps},
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
        "  --verify-warmup SEC       skip this many leading seconds before comparing in --verify\n"
        "                              (default: 10x the config's slowest time constant; 0 to\n"
        "                              compare everything)\n"
        "  --sweep section.key=A,B,C run once per value, printing a summary table\n"
        "  --resample-dt SEC         re-run at a fixed timestep instead of the recorded one\n"
        "  --synth SPEC              synthesise cues instead of reading a file:\n"
        "                              step:<g>:<durSec>\n"
        "                              sine:<hz>:<g>:<durSec>\n"
        "                              chirp:<f0>-<f1>:<g>:<durSec>\n"
        "  --synth-dt SEC            timestep for --synth (default 1/60)\n");
}

// One full pass of the cueing chain over `samples`. Shared by the plain replay
// path, --sweep (run once per swept value) and --verify (run once, compared
// against the recording's live_* columns). Pulling this out of main() is what
// lets --sweep re-run the whole chain per value without duplicating the loop.
struct RunResult {
    size_t samples = 0;
    double durationSec = 0.0;
    double maxLiveErr = 0.0;   // max |replayed - recorded| over the live_* columns, post warm-up
    bool   liveErrIsNaN = false;  // a NaN divergence was seen; maxLiveErr is a sentinel (inf), not a magnitude
    double satHeavePct = 0.0;  // % of active (unpaused) ticks with the heave clamp engaged; NaN if there were none
    double peakHeaveRawMm = 0.0;
    // --verify warm-up accounting (see verifyWarmupSec on runChain). Counted
    // only among samples that carry recorded live_* columns at all -- see
    // haveRecorded below.
    size_t verifySkippedSamples  = 0;
    double verifySkippedSec      = 0.0;
    size_t verifyComparedSamples = 0;

    // Change 1 -- discriminating a decaying initial condition from a real
    // divergence. See the PASS/FAIL logic in main().
    //
    // Residual over the back half of the COMPARED window (i.e. still after
    // the warm-up skip above). An unrecorded initial condition must decay to
    // (and stay within) the noise floor well before EOF; a real difference
    // does not.
    double finalHalfMaxErr  = 0.0;
    bool   finalHalfIsNaN   = false;
    size_t finalHalfSamples = 0;
    // Smallest warm-up, measured from t=0 (independent of whatever
    // --verify-warmup was actually requested), that would have made the OLD
    // criterion -- "residual under the floor for every sample from the
    // warm-up to EOF" -- pass outright. 0 if the residual never exceeds the
    // floor at all. Reported alongside a PASS so a decayed-transient result
    // never hides how much of the file the naive criterion would have had to
    // discard.
    double effectiveWarmupSec = 0.0;
    // Guard against the hazard a pure two-halves check can't see by
    // construction: a difference that isn't actually decaying, but merely
    // stops being EXCITED for a while (measured concretely on the ground
    // segments -- EffectsLayer's onGround-gated rumble oscillator diverges
    // hard while on the ground, then reads as an exact 0.0 the instant the
    // aircraft leaves the ground, for the rest of the file). That looks
    // identical to "already decayed" if only the back half is inspected. A
    // genuine homogeneous decay (the filters' own linear response to a wrong
    // initial STATE) can only ever get smaller over time; regrowth above the
    // floor anywhere in the compared window, after having been smaller, is
    // proof it is not one. See the bucket scan in runChain.
    bool   monotonicViolation      = false;
    double monotonicViolationAtSec = 0.0;
};

RunResult runChain(const std::vector<CueSample>& samples,
                   const StewartGeometry& geo, const WashoutConfig& wcfg,
                   const EffectsConfig& ecfg, const SafetyConfig& scfg,
                   double resampleDt, Telemetry* out,
                   double verifyWarmupSec = 0.0) {
    WashoutFilter     washout(wcfg);
    EffectsLayer      effects(ecfg);
    StewartKinematics kin(geo);
    SafetyLimiter     safety(scfg);

    // Change 2: seed the effects layer's state from the recording's own first
    // row, when it carries the eff_* state columns. rumblePhase_ is a
    // free-running oscillator with no decay of its own -- a warm-up window
    // can't wash out an unrecorded phase the way it washes out a filter's
    // initial condition, so unlike the washout/limiter seeds above, this one
    // has to come from the recording verbatim. Every one of this campaign's
    // seven real recordings predates these columns, so haveEffState is false
    // for all of them and effects starts from its all-zero default -- exactly
    // today's (pre-Change-2) behaviour.
    if (!samples.empty() && samples.front().haveEffState) {
        effects.setState(samples.front().effState);
    }

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
    // Per-recorded-sample error series, from t=0 -- i.e. NOT limited to the
    // post-warm-up "compared" window -- so the Change-1 discrimination below
    // (effectiveWarmupSec, the final-half split, the monotonicity guard) has
    // the whole file to work with. Index i here always corresponds to the i-th
    // sample with s.haveRecorded true, same order the skip/compare counters
    // below walk in, which is what lets cmpStart below be
    // res.verifySkippedSamples without re-deriving it.
    std::vector<double> allTs, allErrs;
    // Both are held across paused ticks, mirroring MotionProvider's
    // lastLivePose_ and lastEffectsPose_ — the plugin holds them rather than
    // zeroing them, and a zero-dip in the middle of a recording would read as a
    // real signal to anyone inspecting the CSV later.
    Pose live;
    Pose e;

    // MotionProvider::onFlightLoopTick resets the stateful filters on the
    // rising edge out of ArmState::Disarmed ("reset so the ramp starts from a
    // clean pose"). runChain must mirror that, or any recording spanning a
    // disarmed->armed transition diverges from that tick onward and --verify
    // fails for a reason that has nothing to do with the filters.
    //
    // The reset is applied ONE SAMPLE LATE, deliberately. Within a plugin tick
    // the order is: washout_->update() (top of the tick) ... washout_->reset()
    // (the arm-intent block, ~40 lines later) ... armRamp_.update() ...
    // telemetry write with row.armState = armRamp_.state(). So the first row
    // whose arm_state leaves 0 was still computed from the OLD filter state,
    // and the reset first shows in the NEXT row. Resetting before the edge
    // sample would shift everything by one tick and break the very comparison
    // this models.
    //
    // A file with no arm_state column (a synthetic stream, a cue-only export)
    // simply carries no edges -- that is a valid input, not an error.
    int  prevArm      = -1;   // -1 = nothing seen yet, so sample 0 is never an edge
    bool pendingReset = false;

    for (const CueSample& s : samples) {
        if (pendingReset) { washout.reset(); effects.reset(); pendingReset = false; }
        const double dtRaw = (resampleDt > 0.0) ? resampleDt : s.dt;
        double dt = dtRaw;
        if (dt > scfg.maxDtSec) dt = scfg.maxDtSec;

        // Snapshot BEFORE this tick's effects.update() (if it runs at all --
        // see MotionProvider::onFlightLoopTick's matching snapshot, which
        // this mirrors exactly, including the pendingReset ordering above).
        const EffectsLayer::State effStateThisTick = effects.state();

        // The plugin gates ONLY the filter update on pause: the IK solve, the
        // limiter and the telemetry write all run every tick against the held
        // pose (MotionProvider::onFlightLoopTick). Replay must mirror that, or a
        // recording containing a pause replays to fewer rows than it has and the
        // limiter state diverges across the gap — which would break the
        // row-for-row --verify below.
        if (!s.cues.simPaused) {
            const Pose w = washout.update(s.cues, dt);
            e = effects.update(s.cues, dt);
            // All six DOF, matching MotionProvider::onFlightLoopTick exactly. The
            // translational onset channel writes surge/sway, so dropping them here
            // would leave replay emitting a constant zero on two of the columns
            // --verify compares — failing every recording made with the channel
            // enabled, for a reason that has nothing to do with the filter.
            live.surge = w.surge + e.surge;  live.sway  = w.sway  + e.sway;
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
            const double d[6] = {
                std::fabs(static_cast<double>(live.heave) - s.recLiveHeave),
                std::fabs(static_cast<double>(live.roll)  - s.recLiveRoll),
                std::fabs(static_cast<double>(live.pitch) - s.recLivePitch),
                std::fabs(static_cast<double>(live.yaw)   - s.recLiveYaw),
                std::fabs(static_cast<double>(live.surge) - s.recLiveSurge),
                std::fabs(static_cast<double>(live.sway)  - s.recLiveSway)};
            // fabs(NaN) is NaN, and "NaN > sampleErr" is false by IEEE-754, so
            // a naive max-tracking comparison silently drops a NaN divergence
            // instead of ever seeing it. Latch a sentinel (+inf) instead --
            // it also sorts as an unambiguous violation everywhere a
            // threshold or a bucket-to-bucket comparison looks at this value
            // below, with no separate NaN case needed there.
            double sampleErr = 0.0;
            for (double v : d) {
                if (std::isnan(v)) sampleErr = std::numeric_limits<double>::infinity();
                else if (v > sampleErr) sampleErr = v;
            }
            allTs.push_back(t);
            allErrs.push_back(sampleErr);

            // Skip a warm-up window before comparing: the recording may have
            // started with the plugin's filters already settled (Record
            // pressed after the platform had been running for a while), while
            // this chain always starts from zeroed filter state. That is an
            // unrecorded initial condition, not a reproduction failure -- see
            // the --verify-warmup discussion in main(). `t` here is the
            // elapsed sim time at the START of this sample (matches r.t
            // above), so a sample is "within" the warm-up while t is still
            // less than the requested window.
            if (t < verifyWarmupSec) {
                ++res.verifySkippedSamples;
                res.verifySkippedSec += dtRaw;
            } else {
                ++res.verifyComparedSamples;
                if (std::isinf(sampleErr)) res.liveErrIsNaN = true;
                if (sampleErr > res.maxLiveErr) res.maxLiveErr = sampleErr;
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
            r.effState = effStateThisTick;
            out->write(r);
        }
        t += dtRaw;

        // Latch the edge only after this sample is fully processed -- see the
        // one-sample-late note above. Any 0 -> non-zero transition counts:
        // with a very short arm_ramp_sec the ramp can reach Armed(2) within
        // the same tick it left Disarmed, so keying on Arming(1) alone would
        // miss that reset.
        if (s.haveArmState) {
            if (prevArm == 0 && s.armState != 0) pendingReset = true;
            prevArm = s.armState;
        }
    }
    res.samples     = counted;
    res.durationSec = t;
    // No unpaused tick means the heave-saturation percentage is undefined, not
    // zero. 0% is this campaign's TARGET outcome, so reporting it here would
    // make an all-paused (or empty) recording read as a perfect result -- and
    // washout_metrics.py already returns NaN for exactly this case, so a 0.0
    // here would also put the two tools in disagreement.
    if (activeTicks) {
        res.satHeavePct = 100.0 * static_cast<double>(clamped) / static_cast<double>(activeTicks);
    } else {
        res.satHeavePct = std::numeric_limits<double>::quiet_NaN();
        std::fprintf(stderr,
            "warning: no unpaused ticks in this run (%zu rows, all paused or empty) -- "
            "heave saturation is undefined (nan), not 0%%\n", counted);
    }

    // Change 1's discrimination, computed once over allTs/allErrs (see their
    // declaration above). Only meaningful with at least one recorded sample;
    // an all-synthetic/bare-cue run leaves everything at its zero default,
    // which main()'s "no sample carried recorded live_* columns" refusal
    // catches before any of this is read.
    if (!allErrs.empty()) {
        // effectiveWarmupSec: the smallest warm-up, from t=0, that would put
        // every remaining sample under the floor -- i.e. what the OLD
        // criterion (skip N seconds, then require the rest to be under the
        // floor) would have needed to pass this file outright. Found by
        // scanning backward for the LAST sample that still violates the
        // floor; everything after it has been under the floor ever since.
        size_t lastViolation = allErrs.size();  // sentinel: no violation found
        for (size_t i = allErrs.size(); i-- > 0; ) {
            if (allErrs[i] > kVerifyToleranceMmDeg) { lastViolation = i; break; }
        }
        if (lastViolation == allErrs.size()) {
            res.effectiveWarmupSec = 0.0;  // never exceeds the floor at all
        } else if (lastViolation + 1 < allErrs.size()) {
            res.effectiveWarmupSec = allTs[lastViolation + 1];
        } else {
            res.effectiveWarmupSec = allTs[lastViolation];  // violates through EOF
        }

        // The final-half split and the monotonicity guard both work over the
        // COMPARED window only (i.e. still after the configured/default
        // warm-up skip) -- allTs/allErrs are pushed in the same order and
        // under the same s.haveRecorded condition as verifySkippedSamples/
        // verifyComparedSamples above, so that suffix starts exactly at index
        // res.verifySkippedSamples.
        const size_t cmpStart = res.verifySkippedSamples;
        const size_t cmpCount = res.verifyComparedSamples;
        if (cmpCount > 0) {
            const size_t half = cmpCount / 2;
            double fhMax = 0.0;
            for (size_t i = cmpStart + half; i < cmpStart + cmpCount; ++i) {
                if (allErrs[i] > fhMax) fhMax = allErrs[i];
            }
            res.finalHalfMaxErr  = fhMax;
            res.finalHalfIsNaN   = std::isinf(fhMax);
            res.finalHalfSamples = cmpCount - half;

            // Guard: partition the SAME compared window into finer buckets
            // and require the per-bucket max to be non-increasing. A genuine
            // initial-condition transient is the filters' own homogeneous
            // response to a wrong starting STATE -- a real, single/multi-pole
            // linear decay -- which can only get smaller with time. Anything
            // that gets bigger again partway through (even if the back half
            // this particular file happens to land on is quiet) is proof the
            // divergence is still being driven by something in the cues, not
            // fading state -- measured concretely on ground_takeoff, where
            // the rumble-phase mismatch (Change 2) is large throughout the
            // ground roll and then reads as an exact 0.0 for the rest of the
            // file simply because onGround went false, not because anything
            // decayed. 20 buckets is fine enough to catch that: it isolates
            // the ground roll (a fraction of the compared window) from the
            // quiet tail that follows, while coarse enough that ordinary
            // float noise near the floor doesn't false-trigger it -- checked
            // against all three of this campaign's real divergent recordings
            // and its one real decaying-transient recording before landing on
            // it (see docs/motion-tuning/README.md's --verify section).
            constexpr int kMonotonicBuckets = 20;
            double prevBucketMax = -1.0;
            for (int k = 0; k < kMonotonicBuckets && !res.monotonicViolation; ++k) {
                const size_t lo = cmpStart + (static_cast<size_t>(k) * cmpCount) / kMonotonicBuckets;
                const size_t hi = cmpStart + (static_cast<size_t>(k + 1) * cmpCount) / kMonotonicBuckets;
                if (lo >= hi) continue;
                double bucketMax = 0.0;
                for (size_t i = lo; i < hi; ++i) if (allErrs[i] > bucketMax) bucketMax = allErrs[i];
                if (prevBucketMax >= 0.0 && bucketMax > prevBucketMax &&
                    bucketMax > kVerifyToleranceMmDeg) {
                    res.monotonicViolation      = true;
                    res.monotonicViolationAtSec = allTs[lo];
                }
                prevBucketMax = bucketMax;
            }
        }
    }
    return res;
}

// Synthetic cue streams, so the filter's response can be characterised with no
// flight at all. SPEC forms:
//   step:<g>:<durSec>            constant g_nrml offset after 1 s
//   sine:<hz>:<g>:<durSec>       sinusoidal g_nrml
//   chirp:<f0>-<f1>:<g>:<durSec> logarithmic sweep, for the frequency response
bool synthCues(const std::string& spec, double dt, std::vector<CueSample>& out,
               std::string& err) {
    std::vector<std::string> p;
    { std::stringstream ss(spec); std::string f;
      while (std::getline(ss, f, ':')) p.push_back(f); }
    if (p.empty()) { err = "empty --synth spec"; return false; }

    const std::string kind = p[0];
    auto sample = [&](double gOffset) {
        CueSample s;
        s.dt = dt;
        s.cues.heaveG = static_cast<float>(1.0 + gOffset);
        return s;
    };

    if (kind == "step") {
        if (p.size() < 3) { err = "step wants step:<g>:<durSec>"; return false; }
        const double g = std::atof(p[1].c_str()), dur = std::atof(p[2].c_str());
        if (dur <= 0.0) { err = "step needs a positive durSec"; return false; }
        if (!std::isfinite(g) || g == 0.0) { err = "step needs a nonzero, finite g"; return false; }
        for (double t = 0.0; t < dur; t += dt) out.push_back(sample(t < 1.0 ? 0.0 : g));
        return true;
    }
    if (kind == "sine") {
        if (p.size() < 4) { err = "sine wants sine:<hz>:<g>:<durSec>"; return false; }
        const double hz = std::atof(p[1].c_str()), g = std::atof(p[2].c_str());
        const double dur = std::atof(p[3].c_str());
        if (dur <= 0.0) { err = "sine needs a positive durSec"; return false; }
        if (hz <= 0.0) { err = "sine needs a positive hz"; return false; }
        if (!std::isfinite(g) || g == 0.0) { err = "sine needs a nonzero, finite g"; return false; }
        for (double t = 0.0; t < dur; t += dt)
            out.push_back(sample(g * std::sin(2.0 * M_PI * hz * t)));
        return true;
    }
    if (kind == "chirp") {
        if (p.size() < 4) { err = "chirp wants chirp:<f0>-<f1>:<g>:<durSec>"; return false; }
        const size_t dash = p[1].find('-');
        if (dash == std::string::npos) { err = "chirp wants f0-f1"; return false; }
        const double f0 = std::atof(p[1].substr(0, dash).c_str());
        const double f1 = std::atof(p[1].c_str() + dash + 1);
        const double g = std::atof(p[2].c_str()), dur = std::atof(p[3].c_str());
        if (dur <= 0.0) { err = "chirp needs a positive durSec"; return false; }
        if (f0 <= 0.0 || f1 <= 0.0) { err = "chirp needs positive f0,f1"; return false; }
        if (f0 == f1) {
            err = "chirp needs two distinct frequencies -- a logarithmic sweep divides by "
                  "log(f1/f0), and f0 == f1 makes that zero (the resulting phase is 0/0 = NaN, "
                  "which would silently pass through as a plausible-looking but garbage result)";
            return false;
        }
        if (!std::isfinite(g) || g == 0.0) { err = "chirp needs a nonzero, finite g"; return false; }
        // Logarithmic sweep: phase is the integral of the instantaneous frequency.
        const double k = std::log(f1 / f0) / dur;
        for (double t = 0.0; t < dur; t += dt) {
            const double phase = 2.0 * M_PI * f0 * (std::exp(k * t) - 1.0) / k;
            out.push_back(sample(g * std::sin(phase)));
        }
        return true;
    }
    err = "unrecognised --synth spec: " + spec;
    return false;
}

}  // namespace

int main(int argc, char** argv) {
    std::string cuesPath, configPath, outPath;
    std::vector<std::pair<std::string, double>> overrides;
    bool                doVerify = false;
    bool                verifyWarmupSet = false;
    double              verifyWarmupArg = 0.0;
    double              resampleDt = 0.0;
    std::string         sweepKey;
    std::vector<double> sweepValues;
    std::string synthSpec;
    double      synthDt = 1.0 / 60.0;

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
        else if (a == "--verify-warmup") {
            verifyWarmupSet = true;
            verifyWarmupArg = std::atof(next("--verify-warmup").c_str());
        }
        else if (a == "--resample-dt") resampleDt = std::atof(next("--resample-dt").c_str());
        else if (a == "--synth")    synthSpec = next("--synth");
        else if (a == "--synth-dt") synthDt   = std::atof(next("--synth-dt").c_str());
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
    if (configPath.empty() || (cuesPath.empty() && synthSpec.empty())) { usage(); return 2; }
    if (!synthSpec.empty() && resampleDt > 0.0) {
        std::fprintf(stderr,
            "REFUSED: --synth and --resample-dt together decouple the filter's clock from the "
            "frequency law baked into the synthetic samples -- pass --synth-dt instead if you "
            "want a different synthetic timestep\n");
        return 2;
    }
    // Every --verify refusal that is decidable from the arguments alone is
    // decided HERE, before anything runs and before a single line reaches
    // stdout. The --set/--resample-dt refusal used to fire only after
    // runChain() had already run and printed its "replayed N samples ..."
    // summary, so an operator saw a normal-looking result line first and the
    // refusal second.
    if (doVerify && !sweepKey.empty()) {
        std::fprintf(stderr, "VERIFY REFUSED: --verify does not run with --sweep\n");
        return 2;
    }
    if (doVerify && (!overrides.empty() || resampleDt != 0.0)) {
        std::fprintf(stderr,
            "VERIFY REFUSED: --verify requires no --set and no --resample-dt\n");
        return 2;
    }
    if (verifyWarmupSet && verifyWarmupArg < 0.0) {
        std::fprintf(stderr, "--verify-warmup must be >= 0 (0 compares everything)\n");
        return 2;
    }

    std::vector<CueSample> samples;
    std::string err;
    if (!synthSpec.empty()) {
        if (!synthCues(synthSpec, synthDt, samples, err)) {
            std::fprintf(stderr, "%s\n", err.c_str()); return 2;
        }
    } else if (!loadCues(cuesPath, samples, err)) {
        std::fprintf(stderr, "%s\n", err.c_str()); return 1;
    }

    bool cfgLoaded = false;
    StewartGeometry geo = MotionConfig::loadGeometry(configPath, &cfgLoaded);
    if (!cfgLoaded) {
        std::fprintf(stderr,
            "cannot parse config %s -- refusing to replay on built-in defaults\n",
            configPath.c_str());
        return 2;
    }
    WashoutConfig   wcfg = MotionConfig::loadWashout(configPath);
    EffectsConfig   ecfg = MotionConfig::loadEffects(configPath);
    SafetyConfig    scfg = MotionConfig::loadSafety(configPath);

    for (const auto& o : overrides) {
        if (!applyOverride(o.first, o.second, wcfg, scfg, ecfg)) {
            std::fprintf(stderr, "unknown key: %s\n", o.first.c_str());
            return 2;
        }
    }

    // --verify's warm-up window: derived from the config's OWN time
    // constants, never hard-coded, because retuning those constants is this
    // campaign's entire purpose -- a fixed window would silently become
    // wrong the moment a candidate changed them. 10x the slowest time
    // constant is comfortably past the point (measured: ~25 s at the shipped
    // 2 s heave washout, itself well under 10x) where an unrecorded initial
    // condition has decayed below the float column's 1e-4 mm resolution.
    // Only computed/used for --verify (--set/--resample-dt/--sweep are
    // already refused together with --verify above, so wcfg here is exactly
    // the config file's values, not an override).
    double verifyWarmupSec = 0.0;
    if (doVerify) {
        if (verifyWarmupSet) {
            verifyWarmupSec = verifyWarmupArg;
        } else {
            double tau = wcfg.heaveHpTau;
            tau = std::max(tau, wcfg.heaveVelWashoutTau);
            tau = std::max(tau, wcfg.heavePosWashoutTau);
            tau = std::max(tau, wcfg.rotHpTau);
            tau = std::max(tau, wcfg.rotWashoutTau);
            tau = std::max(tau, wcfg.tiltLpTau);
            tau = std::max(tau, wcfg.smoothTau);
            verifyWarmupSec = 10.0 * tau;
        }
    }

    if (!sweepKey.empty()) {
        // (--verify + --sweep is refused up in the argument checks, before
        // anything runs.)
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
                // Same failure handling as the plain --out path: an unwritable
                // directory must not print a normal-looking table while
                // silently writing no CSVs at all.
                if (!sweepOut.start(name)) {
                    std::fprintf(stderr, "cannot write %s\n", name);
                    return 1;
                }
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
                                 outPath.empty() ? nullptr : &out, verifyWarmupSec);
    out.stop();
    std::printf("replayed %zu samples (%.1f s), heave saturated %.2f%% of ticks, "
                "peak raw heave %.1f mm\n",
                r.samples, r.durationSec, r.satHeavePct, r.peakHeaveRawMm);

    if (doVerify) {
        // (--set / --resample-dt / --sweep were refused during argument
        // checking, before any of the output above was printed.)
        //
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
        // A recording shorter than its own warm-up cannot be verified -- refuse
        // rather than pass on the handful of rows the skip happened to leave
        // behind. Checked here (after runChain, using the real elapsed-time
        // accounting) rather than against wall-clock row count, since a
        // resampled or variable-dt file wouldn't make row count a reliable
        // proxy for "seconds remaining".
        if (r.verifyComparedSamples < 100) {
            std::fprintf(stderr,
                "VERIFY REFUSED: only %zu samples remain after skipping %.1f s "
                "(%zu samples) of warm-up -- recording too short to verify "
                "(need >= 100 post-warm-up samples; try --verify-warmup 0 or a "
                "longer recording)\n",
                r.verifyComparedSamples, r.verifySkippedSec, r.verifySkippedSamples);
            return 1;
        }
        std::printf("verify: skipped %.1f s (%zu samples) warm-up, compared %zu samples; "
                    "max |replay - recorded| over live_* = %.9g mm/deg\n",
                    r.verifySkippedSec, r.verifySkippedSamples, r.verifyComparedSamples,
                    r.maxLiveErr);
        if (r.liveErrIsNaN) {
            std::fprintf(stderr,
                "VERIFY FAILED: a NaN divergence was encountered in the live_* "
                "comparison (the max above is an inf sentinel, not a magnitude)\n");
            return 1;
        }
        if (r.maxLiveErr > kVerifyToleranceMmDeg) {
            // The overall residual exceeds the floor. That is not
            // automatically a reproduction failure -- an unrecorded initial
            // condition (Record pressed after the filters had already
            // settled) also starts above the floor, then decays; see the
            // design note on kVerifyToleranceMmDeg. Distinguish the two on a
            // property that actually differs between them: an
            // initial-condition mismatch decays and STAYS decayed; a real
            // difference does not (measured: changing heave_gain by 0.1%
            // holds a sustained 0.0455 mm/deg residual to EOF).
            std::printf(
                "verify: final-half residual (last %zu of %zu compared samples) = %.9g mm/deg\n",
                r.finalHalfSamples, r.verifyComparedSamples, r.finalHalfMaxErr);
            if (r.finalHalfIsNaN || r.finalHalfMaxErr > kVerifyToleranceMmDeg) {
                std::fprintf(stderr,
                    "VERIFY FAILED: replay does not reproduce the recording -- the residual is "
                    "still %.9g mm/deg in the final half of the compared window, above the "
                    "%.0e mm/deg noise-floor tolerance, so this is not a decaying initial "
                    "condition\n",
                    r.finalHalfMaxErr, kVerifyToleranceMmDeg);
                return 1;
            }
            // Final-half check alone isn't sufficient -- see runChain's
            // monotonicity guard and the comment on RunResult::
            // monotonicViolation. Caught concretely on this campaign's
            // ground-segment recordings: the effects-layer rumble-phase
            // mismatch (Change 2) diverges hard while on the ground and
            // reads as an exact 0.0 for the rest of the file the instant the
            // aircraft leaves the ground -- not because anything decayed,
            // but because the divergence stopped being excited. A back half
            // that happens to be quiet is not proof of decay by itself.
            if (r.monotonicViolation) {
                std::fprintf(stderr,
                    "VERIFY FAILED: the final-half residual (%.9g mm/deg) is under the noise "
                    "floor, but the residual is not a decaying transient -- it grew back above "
                    "the floor at t=%.1f s after having been smaller earlier in the compared "
                    "window. A genuine initial-condition mismatch can only get smaller over "
                    "time; regrowth means something is still driving the divergence, even if "
                    "this file's tail doesn't happen to excite it\n",
                    r.finalHalfMaxErr, r.monotonicViolationAtSec);
                return 1;
            }
            std::printf(
                "verify: PASS -- residual decays to the noise floor and stays there (an "
                "unrecorded initial condition, not a reproduction defect). Overall residual "
                "%.9g mm/deg; final-half residual %.9g mm/deg (both against the %.0e mm/deg "
                "floor); the old warm-up-only criterion would have needed --verify-warmup "
                "%.1f to pass this file.\n",
                r.maxLiveErr, r.finalHalfMaxErr, kVerifyToleranceMmDeg, r.effectiveWarmupSec);
        } else {
            std::printf("verify: PASS (within %.0e mm/deg floating-point noise floor)\n",
                        kVerifyToleranceMmDeg);
        }
    }
    return 0;
}
