cmake_minimum_required(VERSION 3.15)
project(motion_provider_tests CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(test_kinematics test_kinematics.cpp ../src/StewartKinematics.cpp)
target_include_directories(test_kinematics PRIVATE ../src)

add_executable(test_config test_config.cpp ../src/MotionConfig.cpp)
target_include_directories(test_config PRIVATE ../src ../third_party/tomlplusplus)

add_executable(test_washout test_washout.cpp ../src/WashoutFilter.cpp)
target_include_directories(test_washout PRIVATE ../src)

add_executable(test_effects test_effects.cpp ../src/EffectsLayer.cpp)
target_include_directories(test_effects PRIVATE ../src)

add_executable(test_bff test_bff.cpp ../src/BffEncoder.cpp)
target_include_directories(test_bff PRIVATE ../src)

add_executable(test_safety test_safety.cpp ../src/SafetyLimiter.cpp ../src/StewartKinematics.cpp)
target_include_directories(test_safety PRIVATE ../src)

add_executable(test_monitor test_monitor.cpp ../src/SafetyMonitor.cpp)
target_include_directories(test_monitor PRIVATE ../src)

add_executable(test_heartbeat test_heartbeat.cpp ../src/HeartbeatDecoder.cpp)
target_include_directories(test_heartbeat PRIVATE ../src)

add_executable(test_armramp test_armramp.cpp)
target_include_directories(test_armramp PRIVATE ../src)

enable_testing()
add_test(NAME kinematics COMMAND test_kinematics)
add_test(NAME config COMMAND test_config)
add_test(NAME washout COMMAND test_washout)
add_test(NAME effects COMMAND test_effects)
add_test(NAME bff COMMAND test_bff)
add_test(NAME safety COMMAND test_safety)
add_test(NAME monitor COMMAND test_monitor)
add_test(NAME heartbeat COMMAND test_heartbeat)
add_test(NAME armramp COMMAND test_armramp)
