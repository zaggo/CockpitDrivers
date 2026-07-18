#include "MotionProvider.h"
#include "StatusWindow.h"
#include "DataRefManager.h"
#include "XPLMUtilities.h"

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
    // Sample the flight-model inputs every tick (later phases feed the washout
    // filter from this); the window only needs it at ~1 Hz.
    if (dataRefs_) {
        latestCues_ = dataRefs_->sample();
    }

    statusAccumSec_ += elapsedSec;
    if (statusAccumSec_ >= 1.0f) {
        statusAccumSec_ = 0.0f;
        if (statusWindow_) {
            statusWindow_->update(latestCues_);
        }
    }
}

void MotionProvider::onAircraftLoaded() {
    if (dataRefs_) {
        dataRefs_->onAircraftLoaded();
    }
}
