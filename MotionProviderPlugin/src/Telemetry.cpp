#include "Telemetry.h"
#include <cstdio>

namespace {
// Round-trip-exact formatting. float carries 9 significant decimal digits,
// double 17. The replay self-test compares recomputed values against these,
// so anything shorter would fail it for the wrong reason.
void putF(std::ofstream& o, float v)  { char b[32]; std::snprintf(b, sizeof(b), "%.9g",  static_cast<double>(v)); o << ',' << b; }
void putD(std::ofstream& o, double v) { char b[40]; std::snprintf(b, sizeof(b), "%.17g", v); o << ',' << b; }
void putI(std::ofstream& o, long v)   { o << ',' << v; }
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
           "sl_vel_clip,sl_acc_clip,arm_state";
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

    // First column has no leading comma; every put* adds one, so emit t_sec raw.
    char b[40];
    std::snprintf(b, sizeof(b), "%.17g", r.t);
    out_ << b;

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

    out_ << '\n';
    ++rows_;

    if (r.t - lastFlushT_ >= 1.0) { out_.flush(); lastFlushT_ = r.t; }
}
