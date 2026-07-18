#pragma once
#include <memory>
#include "MotionCues.h"

class StatusWindow;
class DataRefManager;

class MotionProvider {
public:
    MotionProvider();
    ~MotionProvider();

    MotionProvider(const MotionProvider&) = delete;
    MotionProvider& operator=(const MotionProvider&) = delete;

    // Called from Plugin.cpp (X-Plane ABI thread).
    bool initialize();
    void shutdown();
    void onFlightLoopTick(float elapsedSec);
    void onAircraftLoaded();

private:
    std::unique_ptr<StatusWindow> statusWindow_;
    std::unique_ptr<DataRefManager> dataRefs_;

    // Most recent sampled snapshot (updated every 60 Hz tick).
    MotionCues latestCues_;

    // Status window refresh accumulator (~1 Hz), independent of the 60 Hz tick.
    float statusAccumSec_ = 0.0f;
};
