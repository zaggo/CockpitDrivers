#pragma once
#include <string>
#include "StewartGeometry.h"

namespace MotionConfig {
    // Full path to the tuning config (~/.motionprovider.toml).
    std::string defaultPath();

    // Load geometry from a TOML file. Missing file or parse error -> defaults();
    // individual absent keys fall back to their default value.
    StewartGeometry loadGeometry(const std::string& path);
}
