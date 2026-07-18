# Native Windows build script for DCUProviderPlugin

## Problem

`DCUProviderPlugin/build-windows.sh` cross-compiles for Windows from macOS using
MinGW-w64 and `toolchain-mingw64.cmake`. We need a script that builds the plugin
natively on a Windows machine (no cross-compilation), while keeping the existing
cross-compile path available under a new name.

## Scope

- Rename existing cross-compile script, add a new native one, fix a CMake bug the
  new script exposes, update docs/wiring that reference the old script.
- Out of scope: auto-installing toolchains, auto-detecting X-Plane install paths,
  changing the MinGW/macOS build paths' behavior.

## 1. Rename + build-all.sh wiring

- `git mv build-windows.sh build-xc-windows.sh`.
- `build-all.sh` runs on the macOS dev machine and cross-compiles both targets;
  update its Windows-build step to call `build-xc-windows.sh` and read
  `build-xc-windows/output/win.xpl` (was `build-windows/...`).
- The new native `build-windows.sh` keeps `build-windows/` as its own build dir
  name — no collision, since it only ever runs on an actual Windows box (never
  alongside `build-all.sh`, which is mac-only).

## 2. CMakeLists.txt: flatten multi-config output directory

MSVC builds use CMake's Visual Studio generator, which is multi-config. CMake's
documented behavior: `LIBRARY_OUTPUT_DIRECTORY` / `RUNTIME_OUTPUT_DIRECTORY`
get a per-configuration subdirectory appended automatically for multi-config
generators (e.g. `output/Release/win.xpl` instead of `output/win.xpl`), unless
the per-config property variants are also pinned.

Every build script (and `build-all.sh`) expects a flat `output/win.xpl` /
`output/mac.xpl`. Without a fix, the native Windows build silently produces its
artifact in the wrong place and the verify step in the new script fails.

Fix in `CMakeLists.txt`, right after the existing output-directory
`set_target_properties` call (~line 264-267): loop over
`CMAKE_CONFIGURATION_TYPES` and set `LIBRARY_OUTPUT_DIRECTORY_<CONFIG>` /
`RUNTIME_OUTPUT_DIRECTORY_<CONFIG>` to the same flat `${CMAKE_BINARY_DIR}/output`
for each configuration:

```cmake
foreach(OUTPUTCONFIG ${CMAKE_CONFIGURATION_TYPES})
    string(TOUPPER ${OUTPUTCONFIG} OUTPUTCONFIG)
    set_target_properties(DCUProvider PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY_${OUTPUTCONFIG} "${CMAKE_BINARY_DIR}/output"
        LIBRARY_OUTPUT_DIRECTORY_${OUTPUTCONFIG} "${CMAKE_BINARY_DIR}/output"
    )
endforeach()
```

`CMAKE_CONFIGURATION_TYPES` is empty for single-config generators (Makefiles,
used by the existing macOS and MinGW-cross builds), so this loop is a no-op
there — no behavior change for the existing scripts.

No other CMakeLists changes needed: the `WIN32` branch already has a working
non-MINGW (MSVC) path — links `XPLM_64.lib` directly, applies `/W4` — and the
X-Plane SDK headers already `__declspec(dllexport)` the plugin entry points
under MSVC (`XPLMDefs.h`), so no `.def` file or extra export wiring required.

## 3. New `build-windows.sh` (native)

Same shape as `build-macos.sh` (banner, colored steps, prereq checks, clean,
configure, build, verify, report). Differences:

- `$1 == debug` selects CMake config `Debug`, else `Release` (same CLI
  convention as `build-macos.sh`).
- Build dir: `${SCRIPT_DIR}/build-windows`, output dir:
  `${BUILD_DIR}/output` (flat, thanks to the CMakeLists fix above).
- Prerequisite checks:
  - `cmake` on PATH (error + `brew`-style install hint if missing — here a
    winget/download hint).
  - VS C++ toolchain, located via
    `C:\Program Files (x86)\Microsoft Visual Studio\Installer\vswhere.exe`,
    querying `-requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64`.
    If `vswhere.exe` doesn't exist, or the query returns nothing: red error,
    print install guidance (winget command for VS Build Tools 2022 with the
    C++ workload, plus the download URL), exit 1. No silent/automatic install.
- Configure: `cmake .. -A x64 -DXPLANE_SDK="${SCRIPT_DIR}/SDK"` from inside
  `build-windows/`. No `-G` — let CMake auto-select the installed VS generator
  (works without sourcing `vcvars`/needing `cl.exe` on PATH).
- Build: `cmake --build . --config "$BUILD_TYPE" --parallel`.
- Verify: `${OUTPUT_DIR}/win.xpl` exists; on success print size, path, and
  manual X-Plane install instructions (same text as the old cross-compile
  script, minus the "copy to Windows" step since we're already on Windows). On
  failure, red error and exit 1 — matching every other build script's pattern.

## 4. Docs update

- Top-level `CLAUDE.md` (Commands section, ~line 45): split the single
  `./build-windows.sh # cross-compile with MinGW` line into two:
  - `./build-windows.sh        # native build on Windows (MSVC)`
  - `./build-xc-windows.sh     # cross-compile for Windows from macOS (MinGW)`
- `DCUProviderPlugin/CLAUDE.md` (Commands section): same split.
- `DCUProviderPlugin/README.md`: rename the "Windows (Cross-Compile mit
  MinGW)" section to reference `build-xc-windows.sh`, and add a new "Windows
  (nativ, MSVC)" section documenting `build-windows.sh`.

## Testing

No automated test suite for this plugin (per `DCUProviderPlugin/CLAUDE.md`).
Verification is manual:
- Run `./build-xc-windows.sh` on macOS — confirms the rename didn't break the
  existing cross-compile path, and `build-all.sh` still finds its output.
- Run `./build-windows.sh` (and `./build-windows.sh debug`) on a Windows
  machine with VS Build Tools installed — confirms native build produces
  `build-windows/output/win.xpl`.
- Run `./build-windows.sh` on a Windows machine *without* VS Build Tools —
  confirms the prereq check fails cleanly with install guidance instead of a
  raw CMake error.
