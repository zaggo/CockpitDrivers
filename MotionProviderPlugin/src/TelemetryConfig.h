#pragma once
#include <string>

// [telemetry] section of configuration.toml. Recording is a diagnostic aid, so
// it defaults to off -- an unattended flight should never silently fill a disk.
struct TelemetryConfig {
    bool        enabled = false;   // start recording as soon as the plugin loads
    std::string dir     = "";      // output directory; empty = the plugin directory

    static TelemetryConfig defaults() { return TelemetryConfig{}; }
};
