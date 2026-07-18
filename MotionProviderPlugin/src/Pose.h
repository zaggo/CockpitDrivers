#pragma once

// Desired platform pose relative to home. Translations in mm, rotations in deg.
struct Pose {
    float surge = 0.0f;  // +X
    float sway  = 0.0f;  // +Y
    float heave = 0.0f;  // +Z (added on top of home height z0)
    float roll  = 0.0f;  // about X
    float pitch = 0.0f;  // about Y
    float yaw   = 0.0f;  // about Z
};
