#pragma once
#include <string>
#include "StewartGeometry.h"
#include "WashoutConfig.h"
#include "EffectsConfig.h"
#include "SerialConfig.h"
#include "SafetyConfig.h"
#include "TelemetryConfig.h"

namespace MotionConfig {
    // Fallback path if the plugin directory can't be resolved (~/.motionprovider.toml).
    std::string defaultPath();

    // Write a complete configuration.toml (all sections, all default values) to
    // `path`. Used to seed the file when none exists. Returns false if the file
    // could not be written. XPLM-free (std::ofstream only).
    bool writeDefaults(const std::string& path);

    // Load geometry from a TOML file. Missing file or parse error -> defaults();
    // individual absent keys fall back to their default value. Numeric fields
    // accept either integer or float literals. If outLoaded is non-null it is
    // set to true only when a file was actually parsed (false = using defaults).
    StewartGeometry loadGeometry(const std::string& path, bool* outLoaded = nullptr);

    // Load washout config from a TOML file. Missing file, parse error, or absent
    // keys -> defaults; individual fields fall back to their default value.
    // Numeric fields accept either integer or float literals.
    WashoutConfig   loadWashout(const std::string& path);

    // Load effects config from a TOML file. Same contract as loadWashout.
    EffectsConfig   loadEffects(const std::string& path);

    // Load serial config from a TOML file. Same contract as loadWashout.
    SerialConfig    loadSerial(const std::string& path);

    // Load safety config from a TOML file. Same contract as loadWashout.
    SafetyConfig    loadSafety(const std::string& path);

    // Load telemetry config from a TOML file. Same contract as loadWashout.
    TelemetryConfig loadTelemetry(const std::string& path);
}
