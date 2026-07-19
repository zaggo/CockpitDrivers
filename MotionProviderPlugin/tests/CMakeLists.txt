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

enable_testing()
add_test(NAME kinematics COMMAND test_kinematics)
add_test(NAME config COMMAND test_config)
add_test(NAME washout COMMAND test_washout)
