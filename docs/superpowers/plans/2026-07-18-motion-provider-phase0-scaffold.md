# MotionProviderPlugin — Phase 0 (Scaffold) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Stand up a new `MotionProviderPlugin/` X-Plane 12 plugin that builds with the adapted DCUProviderPlugin pipeline, loads in X-Plane, logs to `Log.txt`, and shows an empty status window via a plugin menu item — doing nothing else.

**Architecture:** Copy DCUProviderPlugin's CMake + shell-script build pipeline and X-Plane SDK into a sibling `MotionProviderPlugin/` folder, then reduce it to a minimal three-file skeleton: `Plugin.cpp` (X-Plane ABI + flight loop), `MotionProvider` (orchestrator owning the window), and a trimmed `StatusWindow` (empty floating window + menu toggle). Serial/dataref/IK/washout code is added in later phase plans, not here.

**Tech Stack:** C++17, X-Plane 12 SDK (XPLM400), CMake ≥ 3.15, clang (macOS) / MSVC + MinGW (Windows), plain shell build scripts. No PlatformIO.

## Global Constraints

- C++ standard: **C++17** (`set(CMAKE_CXX_STANDARD 17)`), exactly as DCUProviderPlugin.
- Plugin ABI feature levels: define `XPLM200 XPLM210 XPLM300 XPLM301 XPLM400` all `=1` per platform (copied verbatim from DCUProviderPlugin's `CMakeLists.txt`).
- Build output name per platform: macOS `mac.xpl`, Windows `win.xpl`, Linux `lin.xpl` (`.xpl` suffix, empty prefix).
- The only file allowed to call the X-Plane plugin ABI (`XPluginStart/Stop/Enable/Disable/ReceiveMessage`) is `Plugin.cpp`. Everything else lives behind `MotionProvider`.
- Config persistence path for this plugin: `~/.motionprovider.cfg` (flat `key=value`), distinct from DCUProviderPlugin's `~/.dcuprovider.cfg`.
- Plugin identity strings — Name: `Motion Provider`, Signature: `com.pleasantsoftware.motion.provider`, Description: `6DOF Motion Platform Cueing - X-Plane Plugin`.
- Zero new third-party linked libraries in Phase 0 (toml++ arrives in a later phase, header-only).
- Work happens on the already-created branch `feature/motion-provider-plugin`.
- **Verification model:** Phase 0 has no pure-logic units, so there are no unit tests here. Each task's "test" is: the plugin **builds** via `./build-macos.sh`, and where stated, **loads in X-Plane 12** with the expected `Log.txt` lines / window behavior. Real TDD begins with the math modules in later phase plans.

---

### Task 1: Scaffold the folder, build pipeline, and a do-nothing loadable plugin

**Files:**
- Create dir: `MotionProviderPlugin/`
- Copy (from `DCUProviderPlugin/`): `SDK/`, `.vscode/`, `.gitignore`, `xplm.def`, `toolchain-mingw64.cmake`, `build-macos.sh`, `build-windows.sh`, `build-xc-windows.sh`, `build-all.sh`
- Create: `MotionProviderPlugin/CMakeLists.txt`
- Create: `MotionProviderPlugin/src/Plugin.cpp`

**Interfaces:**
- Consumes: nothing (first task).
- Produces: a buildable CMake target named `MotionProvider` producing `mac.xpl`; the X-Plane ABI entry points in `Plugin.cpp`. Later tasks rely on `Plugin.cpp` holding a `static std::unique_ptr<MotionProvider> gProvider;`.

- [ ] **Step 1: Create the folder and copy the build pipeline + SDK**

Run (from repo root `/Users/zaggo/Developer/CockpitDrivers`):

```bash
mkdir -p MotionProviderPlugin/src
cp -R DCUProviderPlugin/SDK MotionProviderPlugin/SDK
cp -R DCUProviderPlugin/.vscode MotionProviderPlugin/.vscode
cp DCUProviderPlugin/.gitignore MotionProviderPlugin/.gitignore
cp DCUProviderPlugin/xplm.def MotionProviderPlugin/xplm.def
cp DCUProviderPlugin/toolchain-mingw64.cmake MotionProviderPlugin/toolchain-mingw64.cmake
cp DCUProviderPlugin/build-macos.sh MotionProviderPlugin/build-macos.sh
cp DCUProviderPlugin/build-windows.sh MotionProviderPlugin/build-windows.sh
cp DCUProviderPlugin/build-xc-windows.sh MotionProviderPlugin/build-xc-windows.sh
cp DCUProviderPlugin/build-all.sh MotionProviderPlugin/build-all.sh
chmod +x MotionProviderPlugin/*.sh
```

- [ ] **Step 2: Write `MotionProviderPlugin/CMakeLists.txt`**

This is DCUProviderPlugin's `CMakeLists.txt` with the project/target renamed to `MotionProvider` and the source list reduced to Phase 0 files. Write the file with this exact content:

```cmake
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
)
set(HEADERS
    src/MotionProvider.h
    src/StatusWindow.h
    src/ConfigUtils.h
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
```

- [ ] **Step 3: Rename identifiers in the copied build scripts**

In `build-macos.sh`, `build-windows.sh`, `build-xc-windows.sh`, and `build-all.sh`, apply these literal replacements (case-sensitive):

| Find | Replace |
|---|---|
| `DCUProvider` | `MotionProvider` |
| `DCU Provider` | `Motion Provider` |
| `DCU Plugin` | `Motion Plugin` |
| `dcuprovider` | `motionprovider` |
| `mac.xpl` | `mac.xpl` (unchanged — output name stays `mac`) |

Run:

```bash
cd MotionProviderPlugin
sed -i '' -e 's/DCUProvider/MotionProvider/g' -e 's/DCU Provider/Motion Provider/g' -e 's/DCU Plugin/Motion Plugin/g' -e 's/dcuprovider/motionprovider/g' build-macos.sh build-windows.sh build-xc-windows.sh build-all.sh
cd ..
```

Then review the hardcoded absolute X-Plane install paths (they point at the DCUProvider install location on a specific volume) and update them to your machine's X-Plane 12 path with a `MotionProvider` plugin folder. In `build-macos.sh` these are the `XPLANE_PLUGINS_DIR=` line and the `INSTALL_PATH=` line; in `build-windows.sh` the `INSTALL_PATH=` line. Example for `build-macos.sh`:

```bash
# was: .../plugins/DCUProvider/mac_x64  and  .../plugins/DCUProvider/64/mac.xpl
XPLANE_PLUGINS_DIR="/Volumes/1TBSSD/XPlane/X-Plane 12/Resources/plugins/MotionProvider/mac_x64"
# ...
INSTALL_PATH="/Volumes/1TBSSD/XPlane/X-Plane 12/Resources/plugins/MotionProvider/64/mac.xpl"
```

- [ ] **Step 4: Write a minimal `MotionProviderPlugin/src/Plugin.cpp` that links against a `MotionProvider` type**

This mirrors DCUProviderPlugin's `Plugin.cpp` but with the motion identity strings. It references `MotionProvider` (created in Task 2), so it will not link yet — that is expected and resolved in Task 2. Write:

```cpp
#include "MotionProvider.h"
#include "XPLMPlugin.h"
#include "XPLMProcessing.h"
#include "XPLMUtilities.h"
#include <memory>
#include <cstring>

static std::unique_ptr<MotionProvider> gProvider;

// Fixed motion-output tick, decoupled from render frame rate. 60 Hz target.
static constexpr float kFlightLoopIntervalSec = 1.0f / 60.0f;

static float FlightLoopCB(float elapsedTime, float, int, void*) {
    if (gProvider) {
        gProvider->onFlightLoopTick(elapsedTime);
    }
    return kFlightLoopIntervalSec;
}

PLUGIN_API int XPluginStart(char* outName, char* outSig, char* outDesc) {
    std::strcpy(outName, "Motion Provider");
    std::strcpy(outSig,  "com.pleasantsoftware.motion.provider");
    std::strcpy(outDesc, "6DOF Motion Platform Cueing - X-Plane Plugin");

    gProvider = std::make_unique<MotionProvider>();
    if (!gProvider->initialize()) {
        XPLMDebugString("MotionProvider: Failed to initialize\n");
        gProvider.reset();
        return 0;
    }

    XPLMRegisterFlightLoopCallback(FlightLoopCB, kFlightLoopIntervalSec, nullptr);
    XPLMDebugString("MotionProvider: Plugin started successfully\n");
    return 1;
}

PLUGIN_API void XPluginStop(void) {
    XPLMDebugString("MotionProvider: Plugin stopping\n");
    XPLMUnregisterFlightLoopCallback(FlightLoopCB, nullptr);
    if (gProvider) {
        gProvider->shutdown();
        gProvider.reset();
    }
    XPLMDebugString("MotionProvider: Plugin stopped\n");
}

PLUGIN_API void XPluginDisable(void) {
    XPLMDebugString("MotionProvider: Plugin disabled\n");
}

PLUGIN_API int XPluginEnable(void) {
    XPLMDebugString("MotionProvider: Plugin enabled\n");
    return 1;
}

PLUGIN_API void XPluginReceiveMessage(XPLMPluginID inFromWho, int inMessage, void* inParam) {
    (void)inFromWho;
    (void)inParam;
    if (inMessage == XPLM_MSG_PLANE_LOADED) {
        if (gProvider) {
            gProvider->onAircraftLoaded();
        }
    }
}
```

- [ ] **Step 5: Verify it does NOT build yet (missing `MotionProvider`)**

Run:

```bash
cd MotionProviderPlugin && ./build-macos.sh ; cd ..
```

Expected: **build FAILS** at compile/link with an error about `MotionProvider.h` not found (or unresolved `MotionProvider`). This confirms the pipeline runs and reaches compilation. Task 2 makes it pass.

---

### Task 2: MotionProvider orchestrator + flight-loop wiring

**Files:**
- Create: `MotionProviderPlugin/src/MotionProvider.h`
- Create: `MotionProviderPlugin/src/MotionProvider.cpp`

**Interfaces:**
- Consumes: `Plugin.cpp`'s calls — `initialize() -> bool`, `shutdown() -> void`, `onFlightLoopTick(float elapsedSec) -> void`, `onAircraftLoaded() -> void`.
- Produces: the `MotionProvider` class. Later tasks (and phases) add owned components (StatusWindow in Task 3; DataRefManager, kinematics, etc. in later phases) and a `changePort()` method. Phase 0 keeps a 1 Hz status-window update accumulator in `onFlightLoopTick`.

- [ ] **Step 1: Write `MotionProvider.h`**

```cpp
#pragma once
#include <memory>

class StatusWindow;

class MotionProvider {
public:
    MotionProvider();
    ~MotionProvider();

    MotionProvider(const MotionProvider&) = delete;
    MotionProvider& operator=(const MotionProvider&) = delete;

    // Called from Plugin.cpp (X-Plane ABI thread).
    bool initialize();
    void shutdown();
    void onFlightLoopTick(float elapsedSec);
    void onAircraftLoaded();

private:
    std::unique_ptr<StatusWindow> statusWindow_;

    // Status window refresh accumulator (~1 Hz), independent of the 60 Hz tick.
    float statusAccumSec_ = 0.0f;
};
```

- [ ] **Step 2: Write `MotionProvider.cpp` (Phase 0: owns the window, ticks it at 1 Hz)**

```cpp
#include "MotionProvider.h"
#include "StatusWindow.h"
#include "XPLMUtilities.h"

MotionProvider::MotionProvider() = default;
MotionProvider::~MotionProvider() = default;

bool MotionProvider::initialize() {
    statusWindow_ = std::make_unique<StatusWindow>();
    statusWindow_->initialize();
    XPLMDebugString("MotionProvider: initialized\n");
    return true;
}

void MotionProvider::shutdown() {
    if (statusWindow_) {
        statusWindow_->destroy();
        statusWindow_.reset();
    }
}

void MotionProvider::onFlightLoopTick(float elapsedSec) {
    statusAccumSec_ += elapsedSec;
    if (statusAccumSec_ >= 1.0f) {
        statusAccumSec_ = 0.0f;
        if (statusWindow_) {
            statusWindow_->update();
        }
    }
}

void MotionProvider::onAircraftLoaded() {
    // No datarefs to resolve yet (Phase 1).
}
```

- [ ] **Step 3: Build (still fails — StatusWindow not written yet)**

Run:

```bash
cd MotionProviderPlugin && ./build-macos.sh ; cd ..
```

Expected: **build FAILS** with `StatusWindow.h` not found. Task 3 resolves it. (This step exists so the implementer confirms the failure moved past `MotionProvider`.)

---

### Task 3: Empty StatusWindow + menu item + visibility persistence (Phase 0 final deliverable)

**Files:**
- Create: `MotionProviderPlugin/src/StatusWindow.h`
- Create: `MotionProviderPlugin/src/StatusWindow.cpp`
- Create: `MotionProviderPlugin/src/ConfigUtils.h`
- Create: `MotionProviderPlugin/src/ConfigUtils.cpp`

**Interfaces:**
- Consumes: `MotionProvider`'s calls — `StatusWindow()`, `initialize() -> void`, `destroy() -> void`, `update() -> void`, `setVisible(bool)`, `isVisible() -> bool`.
- Produces: an empty floating window toggled by a `Plugins > Motion Provider > Show Status Window` menu item. Later phases extend `update()` to take a data struct and add cue/setpoint lines, port chooser, and manual-DOF keys. `ConfigUtils` exposes `loadStatusWindowVisible()`, `saveStatusWindowVisible(bool)`, `getConfigFilePath()`, `loadLastUsedPort()`, `saveLastUsedPort(const std::string&)`.

- [ ] **Step 1: Write `ConfigUtils.h`**

```cpp
#pragma once
#include <string>

std::string getConfigFilePath();
std::string loadLastUsedPort();
void saveLastUsedPort(const std::string& port);
bool loadStatusWindowVisible();
void saveStatusWindowVisible(bool visible);
```

- [ ] **Step 2: Write `ConfigUtils.cpp` (copy of DCUProviderPlugin's, path changed to `~/.motionprovider.cfg`)**

```cpp
#include "ConfigUtils.h"
#include <fstream>
#include <cstdlib>
#include <string>
#include <map>

std::string getConfigFilePath() {
    const char* home = std::getenv("HOME");
    if (!home) return ".motionprovider.cfg";
    return std::string(home) + "/.motionprovider.cfg";
}

static std::map<std::string, std::string> loadConfig() {
    std::map<std::string, std::string> config;
    std::ifstream f(getConfigFilePath());
    if (f.is_open()) {
        std::string line;
        while (std::getline(f, line)) {
            size_t pos = line.find('=');
            if (pos != std::string::npos) {
                config[line.substr(0, pos)] = line.substr(pos + 1);
            }
        }
    }
    return config;
}

static void saveConfig(const std::map<std::string, std::string>& config) {
    std::ofstream f(getConfigFilePath());
    if (f.is_open()) {
        for (const auto& kv : config) {
            f << kv.first << "=" << kv.second << std::endl;
        }
    }
}

std::string loadLastUsedPort() {
    auto config = loadConfig();
    auto it = config.find("port");
    return (it != config.end()) ? it->second : "";
}

void saveLastUsedPort(const std::string& port) {
    auto config = loadConfig();
    config["port"] = port;
    saveConfig(config);
}

bool loadStatusWindowVisible() {
    auto config = loadConfig();
    auto it = config.find("window_visible");
    if (it == config.end()) return true;
    return it->second == "1" || it->second == "true";
}

void saveStatusWindowVisible(bool visible) {
    auto config = loadConfig();
    config["window_visible"] = visible ? "1" : "0";
    saveConfig(config);
}
```

- [ ] **Step 3: Write `StatusWindow.h` (trimmed — no port/stats members yet)**

```cpp
#pragma once
#include "XPLMDisplay.h"
#include "XPLMMenus.h"
#include "XPLMGraphics.h"
#include <string>

class StatusWindow {
public:
    StatusWindow();
    ~StatusWindow();

    StatusWindow(const StatusWindow&) = delete;
    StatusWindow& operator=(const StatusWindow&) = delete;

    void initialize();
    void destroy();
    void setVisible(bool visible);
    bool isVisible() const;

    // Phase 0: no payload. Later phases pass a data struct.
    void update();

private:
    static void drawCallback(XPLMWindowID inWindowID, void* inRefcon);
    static void keyCallback(XPLMWindowID inWindowID, char inKey, XPLMKeyFlags inFlags,
                            char inVirtualKey, void* inRefcon, int losingFocus);
    static void menuCallback(void* inMenuRef, void* inItemRef);

    void draw();
    void drawString(int x, int y, const std::string& text, float r, float g, float b);

    XPLMWindowID windowId_;
    int menuItemIdx_;
    XPLMMenuID pluginMenuId_;
    bool lastKnownVisible_;
};
```

- [ ] **Step 4: Write `StatusWindow.cpp` (empty window: title line only)**

```cpp
#include "StatusWindow.h"
#include "ConfigUtils.h"
#include "XPLMUtilities.h"

StatusWindow::StatusWindow()
    : windowId_(nullptr), menuItemIdx_(-1), pluginMenuId_(nullptr), lastKnownVisible_(false) {}

StatusWindow::~StatusWindow() {
    destroy();
}

void StatusWindow::initialize() {
    bool shouldBeVisible = loadStatusWindowVisible();

    XPLMCreateWindow_t params = {};
    params.structSize = sizeof(XPLMCreateWindow_t);
    params.left = 100;
    params.top = 500;
    params.right = 500;
    params.bottom = 380;
    params.visible = shouldBeVisible ? 1 : 0;
    params.drawWindowFunc = drawCallback;
    params.handleKeyFunc = keyCallback;
    params.handleMouseClickFunc = [](XPLMWindowID, int, int, XPLMMouseStatus, void*) -> int { return 1; };
    params.handleCursorFunc = nullptr;
    params.handleMouseWheelFunc = nullptr;
    params.handleRightClickFunc = nullptr;
    params.refcon = this;
    params.layer = xplm_WindowLayerFloatingWindows;
    params.decorateAsFloatingWindow = xplm_WindowDecorationRoundRectangle;

    windowId_ = XPLMCreateWindowEx(&params);
    if (!windowId_) {
        XPLMDebugString("MotionProvider: Failed to create status window\n");
        return;
    }
    lastKnownVisible_ = shouldBeVisible;

    pluginMenuId_ = XPLMFindPluginsMenu();
    if (pluginMenuId_) {
        int subMenuIdx = XPLMAppendMenuItem(pluginMenuId_, "Motion Provider", nullptr, 1);
        XPLMMenuID ourMenuId = XPLMCreateMenu("Motion Provider", pluginMenuId_, subMenuIdx,
                                              menuCallback, this);
        XPLMAppendMenuItem(ourMenuId, "Show Status Window", this, 1);
        menuItemIdx_ = subMenuIdx;
        pluginMenuId_ = ourMenuId;
    }
    XPLMDebugString("MotionProvider: Status window initialized\n");
}

void StatusWindow::destroy() {
    if (windowId_) { XPLMDestroyWindow(windowId_); windowId_ = nullptr; }
    if (pluginMenuId_) { XPLMDestroyMenu(pluginMenuId_); pluginMenuId_ = nullptr; }
    menuItemIdx_ = -1;
}

void StatusWindow::setVisible(bool visible) {
    if (!windowId_) return;
    XPLMSetWindowIsVisible(windowId_, visible ? 1 : 0);
    lastKnownVisible_ = visible;
    saveStatusWindowVisible(visible);
}

bool StatusWindow::isVisible() const {
    if (!windowId_) return false;
    return XPLMGetWindowIsVisible(windowId_) != 0;
}

void StatusWindow::update() {
    // Keep persisted visibility in sync with the native close button
    // (round-rectangle decoration has no callback of its own).
    bool nowVisible = isVisible();
    if (nowVisible != lastKnownVisible_) {
        lastKnownVisible_ = nowVisible;
        saveStatusWindowVisible(nowVisible);
    }
}

void StatusWindow::drawCallback(XPLMWindowID inWindowID, void* inRefcon) {
    (void)inWindowID;
    StatusWindow* self = static_cast<StatusWindow*>(inRefcon);
    if (self) self->draw();
}

void StatusWindow::keyCallback(XPLMWindowID, char inKey, XPLMKeyFlags, char, void* inRefcon, int) {
    StatusWindow* self = static_cast<StatusWindow*>(inRefcon);
    if (!self) return;
    if (inKey == 27) self->setVisible(false); // ESC hides
}

void StatusWindow::menuCallback(void* inMenuRef, void* inItemRef) {
    (void)inMenuRef;
    StatusWindow* self = static_cast<StatusWindow*>(inItemRef);
    if (self) self->setVisible(!self->isVisible());
}

void StatusWindow::draw() {
    if (!windowId_) return;
    int left, top, right, bottom;
    XPLMGetWindowGeometry(windowId_, &left, &top, &right, &bottom);
    XPLMDrawTranslucentDarkBox(left, top, right, bottom);
    drawString(left + 10, top - 20, "Motion Provider v0.1 (Phase 0)", 0.8f, 1.0f, 0.8f);
}

void StatusWindow::drawString(int x, int y, const std::string& text, float r, float g, float b) {
    float color[3] = { r, g, b };
    XPLMDrawString(color, x, y, const_cast<char*>(text.c_str()), nullptr, xplmFont_Basic);
}
```

- [ ] **Step 5: Build — now it should succeed**

Run:

```bash
cd MotionProviderPlugin && ./build-macos.sh ; cd ..
```

Expected: **BUILD SUCCESSFUL**, and `MotionProviderPlugin/build-macos/output/mac.xpl` exists.

- [ ] **Step 6: Install and load in X-Plane 12, verify behavior**

Copy the built `mac.xpl` into your X-Plane 12 `Resources/plugins/MotionProvider/<arch>/mac.xpl` (the build script offers to do this). Launch X-Plane 12 and verify all of:
- `Log.txt` contains `MotionProvider: Plugin started successfully` and `MotionProvider: Status window initialized`.
- The menu `Plugins > Motion Provider > Show Status Window` exists.
- Toggling it shows/hides a small empty floating window titled `Motion Provider v0.1 (Phase 0)`.
- Closing the window via its native close button, then reopening X-Plane, restores the last visibility state (persisted to `~/.motionprovider.cfg`).
- No errors or crashes in `Log.txt`.

- [ ] **Step 7: Commit**

```bash
git add MotionProviderPlugin docs/superpowers/plans/2026-07-18-motion-provider-phase0-scaffold.md
git commit -m "feat(motion): Phase 0 scaffold - loadable X-Plane plugin with empty status window"
```

---

## Self-Review

- **Spec coverage (Phase 0 scope):** spec §4 Phase 0 asks for a copied/adapted build pipeline (Task 1), a plugin that loads and does nothing (Tasks 1–2), and an empty status window via menu (Task 3). All covered. Serial/dataref/IK/washout/safety are explicitly out of Phase 0 and belong to later phase plans.
- **Placeholder scan:** every code step contains full file content; no TBD/TODO. The only environment-specific value is the absolute X-Plane install path in the build scripts (Task 1 Step 3), which is inherently machine-specific and flagged with an example.
- **Type consistency:** `MotionProvider` methods (`initialize/shutdown/onFlightLoopTick/onAircraftLoaded`) match between `Plugin.cpp` (Task 1), the header (Task 2 Step 1), and the impl (Task 2 Step 2). `StatusWindow` methods (`initialize/destroy/update/setVisible/isVisible`) match between `MotionProvider.cpp` (Task 2) and `StatusWindow.h/.cpp` (Task 3). `StatusWindow::update()` is deliberately parameterless in Phase 0 (noted in the interface block for later phases to widen).
- **Build-order honesty:** Tasks 1 and 2 intentionally end on a failing build (missing forward-referenced type); this is called out so the implementer is not surprised. The first green build is Task 3 Step 5, which is also the phase deliverable.
