#pragma once
#include <string>

// Returns the path to the config file for the plugin (e.g., ~/.dcuprovider.cfg)
std::string getConfigFilePath();

// Loads the last used port from config file. Returns empty string if not set.
std::string loadLastUsedPort();

// Saves the last used port to config file.
void saveLastUsedPort(const std::string& port);

// Loads the last status window visibility state. Returns true if no saved state (default: visible).
bool loadStatusWindowVisible();

// Saves the status window visibility state.
void saveStatusWindowVisible(bool visible);

#include "AxisCurve.h"

// Response curve for one axis, stored as curve.<axis>.<param> keys.
// Missing or unparseable keys load as 0; values are clamped to range.
AxisCurve loadAxisCurve(const std::string& axis);
void saveAxisCurve(const std::string& axis, const AxisCurve& curve);

// Curve window visibility, per axis: curve_window_visible.<axis>. Default:
// hidden. The legacy single-window key (curve_window_visible) from the
// rudder-only version still applies to "rudder" as a fallback.
bool loadCurveWindowVisible(const std::string& axis);
void saveCurveWindowVisible(const std::string& axis, bool visible);

// Shared toe-brake differential dead zone (curve.brake.diff_dead_zone).
// Missing or unparseable -> 0; clamped to 0..1.
float loadBrakeDiffDeadZone();
void  saveBrakeDiffDeadZone(float v);
