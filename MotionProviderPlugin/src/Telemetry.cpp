#include "Telemetry.h"
#include <charconv>

namespace {
// Round-trip-exact AND locale-independent formatting.
//
// std::to_chars is the only C++ number formatter that is locale-independent
// by definition: it always emits '.' as the decimal point and never applies
// digit grouping. That matters here because this file ships inside a shared
// library loaded into X-Plane. snprintf("%.9g"/"%.17g") honours LC_NUMERIC,
// and stream insertion honours the imbued locale's numpunct -- so on a
// comma-decimal machine, or after X-Plane or any other loaded plugin calls
// setlocale(), every value would gain an embedded comma and the CSV would
// silently grow columns. A recording is written once and cannot be
// regenerated, so a corrupt one is unrecoverable.
//
// to_chars' default (no format/precision argument) is the SHORTEST
// representation that round-trips exactly, so this is at least as precise as
// the "%.9g" / "%.17g" it replaces, and usually shorter. tests/test_telemetry
// asserts the exact round-trip the bit-exact replay self-test rests on.
template <typename T>
void putNum(std::ofstream& o, T v) {
    char b[64];   // shortest round-trip of any double fits in ~24 chars
    const std::to_chars_result r = std::to_chars(b, b + sizeof(b), v);
    o << ',';
    if (r.ec == std::errc()) o.write(b, static_cast<std::streamsize>(r.ptr - b));
    else                     o << '0';   // unreachable with a 64-byte buffer
}

void putF(std::ofstream& o, float v)  { putNum(o, v); }
void putD(std::ofstream& o, double v) { putNum(o, v); }
void putI(std::ofstream& o, long v)   { putNum(o, v); }
}  // namespace

Telemetry::~Telemetry() { stop(); }

const char* Telemetry::header() {
    return "t_sec,dt_real,dt_clamped,"
           "g_nrml,g_axil,g_side,P,Q,R,theta,phi,onground,gs,rpm,alpha,paused,"
           "heave_a_hp,heave_vel,heave_pos_raw,heave_clamped,"
           "tilt_pitch,tilt_roll,tilt_rate_active,"
           "rot_roll_raw,rot_pitch_raw,rot_yaw_raw,"
           "rot_roll_clamped,rot_pitch_clamped,rot_yaw_clamped,"
           "eff_heave,eff_roll,eff_pitch,eff_yaw,"
           "live_heave,live_roll,live_pitch,live_yaw,"
           "cmd_heave,cmd_roll,cmd_pitch,cmd_yaw,"
           "reach_scale,"
           "sp0,sp1,sp2,sp3,sp4,sp5,"
           "sent0,sent1,sent2,sent3,sent4,sent5,"
           "sl_vel_clip,sl_acc_clip,arm_state,"
           "eff_prev_onground,eff_td_active,eff_td_t,eff_rumble_phase,"
           "eff_slab_dist,eff_slab_from,eff_slab_to,eff_slab_t,eff_slab_dur"
           ",surge_a_hp,surge_vel,surge_pos_raw,surge_clamped"
           ",sway_a_hp,sway_vel,sway_pos_raw,sway_clamped"
           ",live_surge,live_sway,cmd_surge,cmd_sway";
}

bool Telemetry::start(const std::string& path) {
    stop();
    out_.open(path, std::ios::out | std::ios::trunc);
    if (!out_.is_open()) return false;
    out_ << header() << '\n';
    path_       = path;
    rows_       = 0;
    lastFlushT_ = 0.0;
    return true;
}

void Telemetry::stop() {
    if (out_.is_open()) { out_.flush(); out_.close(); }
}

void Telemetry::write(const TelemetryRow& r) {
    if (!out_.is_open()) return;

    // First column has no leading comma; every put* adds one, so emit t_sec raw
    // (same locale-independent to_chars path as putNum).
    {
        char b[64];
        const std::to_chars_result tr = std::to_chars(b, b + sizeof(b), r.t);
        if (tr.ec == std::errc()) out_.write(b, static_cast<std::streamsize>(tr.ptr - b));
        else                      out_ << '0';
    }

    putD(out_, r.dtReal);        putD(out_, r.dtClamped);

    putF(out_, r.cues.heaveG);   putF(out_, r.cues.surgeG);   putF(out_, r.cues.swayG);
    putF(out_, r.cues.rollRate); putF(out_, r.cues.pitchRate); putF(out_, r.cues.yawRate);
    putF(out_, r.cues.pitchDeg); putF(out_, r.cues.rollDeg);
    putI(out_, r.cues.onGround ? 1 : 0);
    putF(out_, r.cues.groundspeed); putF(out_, r.cues.engineRpm); putF(out_, r.cues.alphaDeg);
    putI(out_, r.cues.simPaused ? 1 : 0);

    putD(out_, r.trace.heaveAHp); putD(out_, r.trace.heaveVel); putD(out_, r.trace.heavePosRaw);
    putI(out_, r.trace.heaveClamped ? 1 : 0);
    putD(out_, r.trace.tiltPitch); putD(out_, r.trace.tiltRoll);
    putI(out_, r.trace.tiltRateActive ? 1 : 0);
    putD(out_, r.trace.rotRollRaw); putD(out_, r.trace.rotPitchRaw); putD(out_, r.trace.rotYawRaw);
    putI(out_, r.trace.rotRollClamped ? 1 : 0);
    putI(out_, r.trace.rotPitchClamped ? 1 : 0);
    putI(out_, r.trace.rotYawClamped ? 1 : 0);

    putF(out_, r.effects.heave);   putF(out_, r.effects.roll);
    putF(out_, r.effects.pitch);   putF(out_, r.effects.yaw);
    putF(out_, r.live.heave);      putF(out_, r.live.roll);
    putF(out_, r.live.pitch);      putF(out_, r.live.yaw);
    putF(out_, r.commanded.heave); putF(out_, r.commanded.roll);
    putF(out_, r.commanded.pitch); putF(out_, r.commanded.yaw);

    putD(out_, r.reachScale);
    for (int i = 0; i < 6; ++i) putI(out_, r.setpoints[i]);
    for (int i = 0; i < 6; ++i) putI(out_, r.sent[i]);
    putI(out_, r.velClips); putI(out_, r.accClips); putI(out_, r.armState);

    putI(out_, r.effState.prevOnGround ? 1 : 0);
    putI(out_, r.effState.tdActive ? 1 : 0);
    putD(out_, r.effState.tdT);
    putD(out_, r.effState.rumblePhase);
    putD(out_, r.effState.slabDist);
    putD(out_, r.effState.slabFrom);
    putD(out_, r.effState.slabTo);
    putD(out_, r.effState.slabT);
    putD(out_, r.effState.slabDur);

    // No eff_surge/eff_sway: EffectsLayer produces no horizontal component,
    // so the eff_* group stays four wide while live_*/cmd_* below go to six.
    putD(out_, r.trace.surgeAHp); putD(out_, r.trace.surgeVel);
    putD(out_, r.trace.surgePosRaw); putI(out_, r.trace.surgeClamped ? 1 : 0);
    putD(out_, r.trace.swayAHp);  putD(out_, r.trace.swayVel);
    putD(out_, r.trace.swayPosRaw);  putI(out_, r.trace.swayClamped ? 1 : 0);
    putF(out_, r.live.surge);      putF(out_, r.live.sway);
    putF(out_, r.commanded.surge); putF(out_, r.commanded.sway);

    out_ << '\n';
    ++rows_;

    if (r.t - lastFlushT_ >= 1.0) { out_.flush(); lastFlushT_ = r.t; }
}
