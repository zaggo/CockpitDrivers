#pragma once
#include <cstdint>
#include <fstream>
#include <string>
#include "MotionCues.h"
#include "Pose.h"
#include "WashoutFilter.h"   // WashoutTrace

// One recorded flight-loop tick. The cue fields alone are what the offline
// replay tool consumes; everything else is measurement output.
struct TelemetryRow {
    double       t          = 0.0;   // seconds since recording started
    double       dtReal     = 0.0;   // unclamped flight-loop timestep
    double       dtClamped  = 0.0;   // timestep the filters actually saw
    MotionCues   cues;
    WashoutTrace trace;
    Pose         effects;            // effects-layer contribution this tick
    Pose         live;               // washout + effects, BEFORE the arm blend
    Pose         commanded;          // pose fed to the IK, after blend and clamp
    double       reachScale = 1.0;   // clampToReachable bisection factor
    uint16_t     setpoints[6] = {0, 0, 0, 0, 0, 0};   // post-IK
    uint16_t     sent[6]      = {0, 0, 0, 0, 0, 0};   // post-SafetyLimiter
    int          velClips  = 0;
    int          accClips  = 0;
    int          armState  = 0;
};

// Buffered CSV writer. XPLM-free by design: the tests and the offline replay
// tool link this directly. Flushes about once a second so a sim crash costs at
// most a second of data without paying an fsync per tick.
class Telemetry {
public:
    ~Telemetry();

    bool start(const std::string& path);   // truncates, writes the header row
    void stop();
    bool recording() const { return out_.is_open(); }

    void write(const TelemetryRow& r);

    const std::string&  path() const { return path_; }
    unsigned long long  rows() const { return rows_; }

    // Column names, in the exact order write() emits values.
    static const char* header();

private:
    std::ofstream      out_;
    std::string        path_;
    unsigned long long rows_        = 0;
    double             lastFlushT_  = 0.0;
};
