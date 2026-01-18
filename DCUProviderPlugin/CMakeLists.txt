cmake_minimum_required(VERSION 3.15)

project(DCUProviderPlugin)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ============ Find X-Plane SDK ============

# Priority order:
# 1. -DXPLANE_SDK command line argument
# 2. XPLANE_SDK environment variable
# 3. Local SDK folder (next to src/)
# 4. Common installation locations

if(NOT XPLANE_SDK)
    # Check local SDK folder
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/SDK")
        set(XPLANE_SDK "${CMAKE_CURRENT_SOURCE_DIR}/SDK")
        message(STATUS "Using local SDK: ${XPLANE_SDK}")
    
    # Check environment variable
    elseif(DEFINED ENV{XPLANE_SDK})
        set(XPLANE_SDK "$ENV{XPLANE_SDK}")
        message(STATUS "Using XPLANE_SDK environment variable: ${XPLANE_SDK}")
    
    # Check common macOS locations
    elseif(APPLE AND EXISTS "${HOME}/Developer/x-plane-sdk")
        set(XPLANE_SDK "${HOME}/Developer/x-plane-sdk")
        message(STATUS "Found SDK in ~/Developer/x-plane-sdk")
    
    elseif(APPLE AND EXISTS "${HOME}/Developer/X-Plane-12-SDK")
        set(XPLANE_SDK "${HOME}/Developer/X-Plane-12-SDK")
        message(STATUS "Found SDK in ~/Developer/X-Plane-12-SDK")
    
    # Check /opt location
    elseif(EXISTS "/opt/xplane-sdk")
        set(XPLANE_SDK "/opt/xplane-sdk")
        message(STATUS "Found SDK in /opt/xplane-sdk")
    
    else()
        message(FATAL_ERROR 
            "X-Plane SDK not found!\n"
            "Try one of:\n"
            "  1. Place SDK folder in: ${CMAKE_CURRENT_SOURCE_DIR}/SDK\n"
            "  2. Set environment: export XPLANE_SDK=/path/to/sdk\n"
            "  3. Pass to cmake: cmake -DXPLANE_SDK=/path/to/sdk .."
        )
    endif()
else()
    message(STATUS "Using XPLANE_SDK from command line: ${XPLANE_SDK}")
endif()

# Verify SDK exists and has required files
if(NOT EXISTS "${XPLANE_SDK}")
    message(FATAL_ERROR "XPLANE_SDK directory not found: ${XPLANE_SDK}")
endif()

if(NOT EXISTS "${XPLANE_SDK}/CHeaders/XPLM/XPLMPlugin.h")
    message(FATAL_ERROR 
        "X-Plane SDK appears incomplete.\n"
        "Missing: ${XPLANE_SDK}/CHeaders/XPLM/XPLMPlugin.h\n"
        "Is this the correct SDK path?"
    )
endif()

message(STATUS "X-Plane SDK verified: ${XPLANE_SDK}")

# X-Plane SDK paths
set(XPLANE_INCLUDE_DIR "${XPLANE_SDK}/CHeaders/XPLM")
set(XPLANE_UTIL_INCLUDE_DIR "${XPLANE_SDK}/CHeaders/Utilities")

# Plugin sources
set(SOURCES
    src/Plugin.cpp
    src/DCUProvider.cpp
    src/SerialPort.cpp
    src/TransportLayer.cpp
    src/DataRefManager.cpp
    src/MessageQueue.cpp
    src/ConnectionManager.cpp
    src/StatusWindow.cpp
)

# Plugin headers
set(HEADERS
    src/DCUProvider.h
    src/SerialPort.h
    src/TransportLayer.h
    src/DataRefManager.h
    src/MessageQueue.h
    src/ConnectionManager.h
    src/StatusWindow.h
)

# Create plugin library
add_library(DCUProvider SHARED ${SOURCES} ${HEADERS})

# Include directories
target_include_directories(DCUProvider PRIVATE
    ${XPLANE_INCLUDE_DIR}
    ${XPLANE_UTIL_INCLUDE_DIR}
    src
)

# ============ PLATFORM DEFINITIONS ============
if(APPLE)
    # macOS
    target_compile_definitions(DCUProvider PRIVATE
        IBM=0
        LIN=0
        __APPLE__=1
        APL=1
        XPLM200=1
        XPLM210=1
        XPLM300=1
        XPLM301=1
        XPLM400=1
    )
    
    set_target_properties(DCUProvider PROPERTIES
        SUFFIX ".xpl"
        PREFIX ""
        OUTPUT_NAME "DCUProvider"
        LINK_FLAGS "-undefined dynamic_lookup"
    )
    
    # Set architecture for macOS (arm64 and x86_64)
    set_target_properties(DCUProvider PROPERTIES
        OSX_ARCHITECTURES "arm64;x86_64"
    )
    
    # macOS frameworks
    target_link_libraries(DCUProvider PRIVATE
        "-framework OpenGL"
        "-framework Cocoa"
    )
    
    # Compiler flags for macOS
    target_compile_options(DCUProvider PRIVATE
        -fPIC
        -fvisibility=hidden
        -Wall
        -Wextra
        -Wno-deprecated
    )
    
elseif(WIN32)
    # Windows
    target_compile_definitions(DCUProvider PRIVATE
        IBM=1
        LIN=0
        APL=0
        _WIN32=1
        _WIN64=1
        XPLM200=1
        XPLM210=1
        XPLM300=1
        XPLM301=1
        XPLM400=1
    )
    
    set_target_properties(DCUProvider PROPERTIES
        SUFFIX ".xpl"
        PREFIX ""
        OUTPUT_NAME "DCUProvider"
    )
    
    # Windows libraries
    target_link_libraries(DCUProvider PRIVATE
        opengl32
        setupapi
    )
    
    # Compiler flags for Windows
    target_compile_options(DCUProvider PRIVATE
        /W4
        /WX
    )
    
elseif(UNIX AND NOT APPLE)
    # Linux
    target_compile_definitions(DCUProvider PRIVATE
        IBM=0
        LIN=1
        APL=0
        XPLM200=1
        XPLM210=1
        XPLM300=1
        XPLM301=1
        XPLM400=1
    )
    
    set_target_properties(DCUProvider PROPERTIES
        SUFFIX ".xpl"
        PREFIX ""
        OUTPUT_NAME "DCUProvider"
    )
    
    # Linux libraries
    target_link_libraries(DCUProvider PRIVATE
        GL
        X11
    )
    
    # Compiler flags for Linux
    target_compile_options(DCUProvider PRIVATE
        -fPIC
        -fvisibility=hidden
        -Wall
        -Wextra
    )
endif()

# Output directory
set_target_properties(DCUProvider PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/output"
)

# X-Plane plugins directory (for auto-install)
if(APPLE)
    set(DEFAULT_XPLANE_PLUGINS_DIR "${HOME}/Library/Preferences/X-Plane 12/Resources/plugins")
elseif(WIN32)
    set(DEFAULT_XPLANE_PLUGINS_DIR "C:/Users/${ENV:USERNAME}/AppData/Roaming/X-Plane 12/Resources/plugins")
elseif(UNIX AND NOT APPLE)
    set(DEFAULT_XPLANE_PLUGINS_DIR "${HOME}/.x-plane_12/Resources/plugins")
endif()

if(NOT XPLANE_PLUGINS_DIR)
    set(XPLANE_PLUGINS_DIR "${DEFAULT_XPLANE_PLUGINS_DIR}")
endif()

message(STATUS "X-Plane plugins directory: ${XPLANE_PLUGINS_DIR}")

# Optional: Copy plugin to X-Plane plugins directory after build
if(EXISTS "${XPLANE_PLUGINS_DIR}")
    add_custom_command(TARGET DCUProvider POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:DCUProvider>
        "${XPLANE_PLUGINS_DIR}/DCUProvider.xpl"
        COMMENT "Copying plugin to X-Plane plugins directory"
    )
endif()

# Optional: Enable testing
enable_testing()

# Optional: Add unit tests (if tests exist)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests")
    add_subdirectory(tests)
endif()