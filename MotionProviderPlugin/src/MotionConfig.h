#pragma once
#include <string>
#include "StewartGeometry.h"

namespace MotionConfig {
    // Full path to the tuning config (~/.motionprovider.toml).
    std::string defaultPath();

    // Load geometry from a TOML file. Missing file or parse error -> defaults();
    // individual absent keys fall back to their default value. Numeric fields
    // accept either integer or float literals. If outLoaded is non-null it is
    // set to true only when a file was actually parsed (false = using defaults).
    StewartGeometry loadGeometry(const std::string& path, bool* outLoaded = nullptr);
}
