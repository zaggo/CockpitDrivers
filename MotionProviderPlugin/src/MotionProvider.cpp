#include "MotionProvider.h"
#include "StatusWindow.h"
#include "XPLMUtilities.h"

MotionProvider::MotionProvider() = default;
MotionProvider::~MotionProvider() = default;

bool MotionProvider::initialize() {
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
}

void MotionProvider::onFlightLoopTick(float elapsedSec) {
    statusAccumSec_ += elapsedSec;
    if (statusAccumSec_ >= 1.0f) {
        statusAccumSec_ = 0.0f;
        if (statusWindow_) {
            statusWindow_->update();
        }
    }
}

void MotionProvider::onAircraftLoaded() {
    // No datarefs to resolve yet (Phase 1).
}
