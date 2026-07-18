cmake_minimum_required(VERSION 3.15)
project(motion_provider_tests CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(test_kinematics
    test_kinematics.cpp
    ../src/StewartKinematics.cpp
)
target_include_directories(test_kinematics PRIVATE ../src)

enable_testing()
add_test(NAME kinematics COMMAND test_kinematics)
