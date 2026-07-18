cmake_minimum_required(VERSION 3.15)

project(MotionProviderPlugin)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ============ Find X-Plane SDK ============
if(NOT XPLANE_SDK)
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/SDK")
        set(XPLANE_SDK "${CMAKE_CURRENT_SOURCE_DIR}/SDK")
        message(STATUS "Using local SDK: ${XPLANE_SDK}")
    elseif(DEFINED ENV{XPLANE_SDK})
        set(XPLANE_SDK "$ENV{XPLANE_SDK}")
        message(STATUS "Using XPLANE_SDK environment variable: ${XPLANE_SDK}")
    elseif(APPLE AND EXISTS "${HOME}/Developer/x-plane-sdk")
        set(XPLANE_SDK "${HOME}/Developer/x-plane-sdk")
    elseif(APPLE AND EXISTS "${HOME}/Developer/X-Plane-12-SDK")
        set(XPLANE_SDK "${HOME}/Developer/X-Plane-12-SDK")
    elseif(EXISTS "/opt/xplane-sdk")
        set(XPLANE_SDK "/opt/xplane-sdk")
    else()
        message(FATAL_ERROR
            "X-Plane SDK not found!\n"
            "  1. Place SDK folder in: ${CMAKE_CURRENT_SOURCE_DIR}/SDK\n"
            "  2. Set environment: export XPLANE_SDK=/path/to/sdk\n"
            "  3. Pass to cmake: cmake -DXPLANE_SDK=/path/to/sdk ..")
    endif()
else()
    message(STATUS "Using XPLANE_SDK from command line: ${XPLANE_SDK}")
endif()

if(NOT EXISTS "${XPLANE_SDK}")
    message(FATAL_ERROR "XPLANE_SDK directory not found: ${XPLANE_SDK}")
endif()
if(NOT EXISTS "${XPLANE_SDK}/CHeaders/XPLM/XPLMPlugin.h")
    message(FATAL_ERROR "X-Plane SDK appears incomplete. Missing: ${XPLANE_SDK}/CHeaders/XPLM/XPLMPlugin.h")
endif()
message(STATUS "X-Plane SDK verified: ${XPLANE_SDK}")

set(XPLANE_INCLUDE_DIR "${XPLANE_SDK}/CHeaders/XPLM")
set(XPLANE_UTIL_INCLUDE_DIR "${XPLANE_SDK}/CHeaders/Utilities")

# Plugin sources (Phase 0 skeleton; grows in later phases)
set(SOURCES
    src/Plugin.cpp
    src/MotionProvider.cpp
    src/StatusWindow.cpp
    src/ConfigUtils.cpp
    src/DataRefManager.cpp
)
set(HEADERS
    src/MotionProvider.h
    src/StatusWindow.h
    src/ConfigUtils.h
    src/DataRefManager.h
    src/MotionCues.h
)

add_library(MotionProvider SHARED ${SOURCES} ${HEADERS})

target_include_directories(MotionProvider PRIVATE
    ${XPLANE_INCLUDE_DIR}
    ${XPLANE_UTIL_INCLUDE_DIR}
    src
)

# ============ PLATFORM DEFINITIONS ============
if(APPLE)
    target_compile_definitions(MotionProvider PRIVATE
        IBM=0 LIN=0 __APPLE__=1 APL=1
        XPLM200=1 XPLM210=1 XPLM300=1 XPLM301=1 XPLM400=1)
    set_target_properties(MotionProvider PROPERTIES
        SUFFIX ".xpl" PREFIX "" OUTPUT_NAME "mac"
        LINK_FLAGS "-undefined dynamic_lookup"
        OSX_ARCHITECTURES "arm64;x86_64")
    target_link_libraries(MotionProvider PRIVATE "-framework OpenGL" "-framework Cocoa")
    target_compile_options(MotionProvider PRIVATE -fPIC -fvisibility=hidden -Wall -Wextra -Wno-deprecated)
    if(CMAKE_BUILD_TYPE STREQUAL "Debug")
        target_compile_options(MotionProvider PRIVATE -g -O0)
    endif()
elseif(WIN32)
    target_compile_definitions(MotionProvider PRIVATE
        IBM=1 LIN=0 APL=0 _WIN32=1 _WIN64=1
        XPLM200=1 XPLM210=1 XPLM300=1 XPLM301=1 XPLM400=1)
    set_target_properties(MotionProvider PROPERTIES SUFFIX ".xpl" PREFIX "" OUTPUT_NAME "win")
    if(MINGW)
        set(XPLM_DEF_FILE "${CMAKE_CURRENT_SOURCE_DIR}/xplm.def")
        set(XPLM_IMPORT_LIB "${CMAKE_CURRENT_BINARY_DIR}/libxplm.a")
        add_custom_command(OUTPUT ${XPLM_IMPORT_LIB}
            COMMAND ${CMAKE_DLLTOOL} -d ${XPLM_DEF_FILE} -l ${XPLM_IMPORT_LIB}
            DEPENDS ${XPLM_DEF_FILE}
            COMMENT "Creating XPLM import library for MinGW")
        add_custom_target(xplm_import_lib DEPENDS ${XPLM_IMPORT_LIB})
        add_dependencies(MotionProvider xplm_import_lib)
        target_link_libraries(MotionProvider PRIVATE
            ${XPLM_IMPORT_LIB} opengl32 setupapi
            -static-libgcc -static-libstdc++ -static
            -Wl,-Bstatic,--whole-archive -lwinpthread -Wl,--no-whole-archive)
        target_compile_options(MotionProvider PRIVATE -Wall -Wextra -fvisibility=hidden)
    else()
        target_link_libraries(MotionProvider PRIVATE
            ${XPLANE_SDK}/Libraries/Win/XPLM_64.lib opengl32 setupapi)
        target_compile_options(MotionProvider PRIVATE /W4)
    endif()
elseif(UNIX AND NOT APPLE)
    target_compile_definitions(MotionProvider PRIVATE
        IBM=0 LIN=1 APL=0
        XPLM200=1 XPLM210=1 XPLM300=1 XPLM301=1 XPLM400=1)
    set_target_properties(MotionProvider PROPERTIES SUFFIX ".xpl" PREFIX "" OUTPUT_NAME "lin")
    target_link_libraries(MotionProvider PRIVATE GL X11)
    target_compile_options(MotionProvider PRIVATE -fPIC -fvisibility=hidden -Wall -Wextra)
endif()

set_target_properties(MotionProvider PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/output"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/output")
foreach(OUTPUTCONFIG ${CMAKE_CONFIGURATION_TYPES})
    string(TOUPPER ${OUTPUTCONFIG} OUTPUTCONFIG)
    set_target_properties(MotionProvider PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY_${OUTPUTCONFIG} "${CMAKE_BINARY_DIR}/output"
        LIBRARY_OUTPUT_DIRECTORY_${OUTPUTCONFIG} "${CMAKE_BINARY_DIR}/output")
endforeach()

if(APPLE)
    set(DEFAULT_XPLANE_PLUGINS_DIR "${HOME}/Library/Preferences/X-Plane 12/Resources/plugins")
elseif(WIN32)
    set(DEFAULT_XPLANE_PLUGINS_DIR "C:/Users/$ENV{USERNAME}/AppData/Roaming/X-Plane 12/Resources/plugins")
elseif(UNIX AND NOT APPLE)
    set(DEFAULT_XPLANE_PLUGINS_DIR "${HOME}/.x-plane_12/Resources/plugins")
endif()
if(NOT XPLANE_PLUGINS_DIR)
    set(XPLANE_PLUGINS_DIR "${DEFAULT_XPLANE_PLUGINS_DIR}")
endif()

if(EXISTS "${XPLANE_PLUGINS_DIR}")
    add_custom_command(TARGET MotionProvider POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:MotionProvider>
        "${XPLANE_PLUGINS_DIR}/MotionProvider.xpl"
        COMMENT "Copying plugin to X-Plane plugins directory")
endif()

enable_testing()
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests")
    add_subdirectory(tests)
endif()
