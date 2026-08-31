cmake_minimum_required(VERSION 3.15)
project(washout_replay CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Links the real filter sources -- there is deliberately no second
# implementation of the cueing chain that could drift from the plugin's.
add_executable(washout_replay
    washout_replay.cpp
    ../src/WashoutFilter.cpp
    ../src/EffectsLayer.cpp
    ../src/StewartKinematics.cpp
    ../src/SafetyLimiter.cpp
    ../src/MotionConfig.cpp
    ../src/Telemetry.cpp)
target_include_directories(washout_replay PRIVATE ../src ../third_party/tomlplusplus)

# Reachable-envelope measurement. Same rule as washout_replay: links the real
# StewartKinematics, no second copy of the geometry.
add_executable(envelope_probe
    envelope_probe.cpp
    ../src/StewartKinematics.cpp
    ../src/MotionConfig.cpp)
target_include_directories(envelope_probe PRIVATE ../src ../third_party/tomlplusplus)
