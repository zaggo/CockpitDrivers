#include "MotionProvider.h"
#include "StatusWindow.h"
#include "DataRefManager.h"
#include "XPLMUtilities.h"
#include <algorithm>

MotionProvider::MotionProvider() = default;
MotionProvider::~MotionProvider() = default;

bool MotionProvider::initialize() {
    dataRefs_ = std::make_unique<DataRefManager>();
    dataRefs_->initialize();

    statusWindow_ = std::make_unique<StatusWindow>();
    statusWindow_->initialize();

    XPLMDebugString("MotionProvider: initialized\n");
    return true;
}

void MotionProvider::shutdown() {
    if (statusWindow_) {
        statusWindow_->destroy();
        statusWindow_.reset();
    }
    dataRefs_.reset();
}

void MotionProvider::onFlightLoopTick(float elapsedSec) {
    if (dataRefs_) {
        latestCues_ = dataRefs_->sample();
    }

    // Placeholder pose: map aircraft attitude straight to platform tilt, clamped
    // to a small range so it stays reachable. Replaced by the washout filter in
    // Phase 3 - this only exists so Phase 2 shows the IK responding in flight.
    auto clampf = [](float v, float lo, float hi) {
        return std::max(lo, std::min(hi, v));
    };
    Pose pose;
    pose.roll  = clampf(latestCues_.rollDeg,  -8.0f, 8.0f);
    pose.pitch = clampf(latestCues_.pitchDeg, -8.0f, 8.0f);
    latestSolve_ = StewartKinematics::solve(pose);

    statusAccumSec_ += elapsedSec;
    if (statusAccumSec_ >= 1.0f) {
        statusAccumSec_ = 0.0f;
        if (statusWindow_) {
            statusWindow_->update(latestCues_, latestSolve_);
        }
    }
}

void MotionProvider::onAircraftLoaded() {
    if (dataRefs_) {
        dataRefs_->onAircraftLoaded();
    }
}
