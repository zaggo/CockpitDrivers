cmake_minimum_required(VERSION 3.15)
project(dcu_provider_tests CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(test_axiscurve test_axiscurve.cpp ../src/AxisCurve.cpp)
target_include_directories(test_axiscurve PRIVATE ../src)

add_executable(test_config test_config.cpp ../src/ConfigUtils.cpp ../src/AxisCurve.cpp)
target_include_directories(test_config PRIVATE ../src)

enable_testing()
add_test(NAME axiscurve COMMAND test_axiscurve)
add_test(NAME config COMMAND test_config)
