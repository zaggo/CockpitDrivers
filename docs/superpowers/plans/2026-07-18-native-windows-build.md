# Native Windows Build Script Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Split `DCUProviderPlugin/build-windows.sh` into a renamed cross-compile script (`build-xc-windows.sh`, unchanged behavior) and a new `build-windows.sh` that builds the plugin natively on a Windows box via MSVC, no cross-compiling.

**Architecture:** Shell scripts only, mirroring the existing `build-macos.sh` structure (banner, colored prereq checks, clean, CMake configure, build, verify). One CMakeLists.txt fix (flatten multi-config output dir) is required for the new script's verify step to find its artifact. No source/C++ changes.

**Tech Stack:** Bash (Git Bash on Windows), CMake, MSVC (Visual Studio Build Tools) for the new native path; MinGW-w64 for the renamed cross-compile path (untouched).

## Global Constraints

- No automated test suite exists for this plugin (per `DCUProviderPlugin/CLAUDE.md`) — verification throughout this plan is manual: syntax checks (`bash -n`), running scripts, and reading diffs.
- `CMAKE_CONFIGURATION_TYPES`-based fix must be a no-op for single-config generators (Makefiles) — do not change macOS/MinGW build behavior.
- New `build-windows.sh` must never silently auto-install anything; on missing prerequisites it prints install guidance and exits 1.
- Keep the new script's CLI convention identical to `build-macos.sh`: no arg = Release, `debug` arg = Debug config.
- Repo root for all paths below is `c:\Users\zaggo\Developer\CockpitDrivers` (git repo root); the plugin lives in `DCUProviderPlugin/`.

---

### Task 1: Rename cross-compile script, rewire `build-all.sh`

**Files:**
- Rename: `DCUProviderPlugin/build-windows.sh` → `DCUProviderPlugin/build-xc-windows.sh`
- Modify: `DCUProviderPlugin/build-xc-windows.sh:21` (build dir variable)
- Modify: `DCUProviderPlugin/build-all.sh:60-72`

**Interfaces:**
- Produces: `build-xc-windows.sh` — same CLI/behavior as old `build-windows.sh` (cross-compile via MinGW, writes `build-xc-windows/output/win.xpl`). Later tasks (docs) reference this filename.

- [ ] **Step 1: Rename the script with git mv**

```bash
cd DCUProviderPlugin
git mv build-windows.sh build-xc-windows.sh
```

- [ ] **Step 2: Rename the build dir variable inside the renamed script**

In `DCUProviderPlugin/build-xc-windows.sh`, change:

```bash
# Build directory
BUILD_DIR="${SCRIPT_DIR}/build-windows"
```

to:

```bash
# Build directory
BUILD_DIR="${SCRIPT_DIR}/build-xc-windows"
```

- [ ] **Step 3: Update `build-all.sh`'s Windows step to call the renamed script**

In `DCUProviderPlugin/build-all.sh`, replace:

```bash
if [ -f "${SCRIPT_DIR}/build-windows.sh" ]; then
    "${SCRIPT_DIR}/build-windows.sh"
    
    if [ -f "${SCRIPT_DIR}/build-windows/output/win.xpl" ]; then
        cp "${SCRIPT_DIR}/build-windows/output/win.xpl" "${PLUGIN_64_DIR}/win.xpl"
        echo -e "${GREEN}✓ Windows plugin copied to ${PLUGIN_64_DIR}/win.xpl${NC}"
    else
        echo -e "${RED}✗ Windows build failed - plugin not found${NC}"
        exit 1
    fi
else
    echo -e "${RED}✗ build-windows.sh not found${NC}"
    exit 1
fi
```

with:

```bash
if [ -f "${SCRIPT_DIR}/build-xc-windows.sh" ]; then
    "${SCRIPT_DIR}/build-xc-windows.sh"
    
    if [ -f "${SCRIPT_DIR}/build-xc-windows/output/win.xpl" ]; then
        cp "${SCRIPT_DIR}/build-xc-windows/output/win.xpl" "${PLUGIN_64_DIR}/win.xpl"
        echo -e "${GREEN}✓ Windows plugin copied to ${PLUGIN_64_DIR}/win.xpl${NC}"
    else
        echo -e "${RED}✗ Windows build failed - plugin not found${NC}"
        exit 1
    fi
else
    echo -e "${RED}✗ build-xc-windows.sh not found${NC}"
    exit 1
fi
```

- [ ] **Step 4: Syntax-check both scripts**

Run: `bash -n DCUProviderPlugin/build-xc-windows.sh && bash -n DCUProviderPlugin/build-all.sh && echo OK`
Expected: `OK`

- [ ] **Step 5: Verify no stale references remain**

Run: `grep -rn "build-windows" DCUProviderPlugin/build-xc-windows.sh DCUProviderPlugin/build-all.sh`
Expected: no output (the only matches left in the repo should be the new native `build-windows.sh`, created in Task 3, and doc mentions handled in Task 4 — neither exists yet at this point, so this specific grep returns nothing).

- [ ] **Step 6: Commit**

```bash
git add DCUProviderPlugin/build-xc-windows.sh DCUProviderPlugin/build-all.sh
git rm --cached DCUProviderPlugin/build-windows.sh 2>/dev/null; true
git status
git commit -m "$(cat <<'EOF'
refactor: rename cross-compile Windows script to build-xc-windows.sh

Frees up build-windows.sh for a native (non-cross-compiling) build
script, added in a follow-up commit.
EOF
)"
```

(The `git rm --cached` line is a no-op safety net — `git mv` already staged the rename; skip if `git status` shows the rename cleanly staged.)

---

### Task 2: CMakeLists.txt — flatten multi-config output directory

**Files:**
- Modify: `DCUProviderPlugin/CMakeLists.txt:264-267`

**Interfaces:**
- Produces: guarantee that `${CMAKE_BINARY_DIR}/output/<name>.xpl` is the artifact path regardless of generator (single- or multi-config). Task 3's new script relies on this for its verify step (`${OUTPUT_DIR}/win.xpl`, no `Release`/`Debug` subfolder).

- [ ] **Step 1: Locate the current output-directory block**

Read `DCUProviderPlugin/CMakeLists.txt` around line 264 — it currently reads:

```cmake
# Output directory
set_target_properties(DCUProvider PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/output"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/output"
)
```

- [ ] **Step 2: Add the per-config pin right after it**

Replace the block above with:

```cmake
# Output directory
set_target_properties(DCUProvider PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/output"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/output"
)

# Multi-config generators (e.g. Visual Studio) append a per-configuration
# subdirectory to LIBRARY/RUNTIME_OUTPUT_DIRECTORY unless the per-config
# property variants are pinned too. Flatten to a single output/ dir so
# every build script finds the artifact at the same path regardless of
# generator. No-op for single-config generators (Makefiles).
foreach(OUTPUTCONFIG ${CMAKE_CONFIGURATION_TYPES})
    string(TOUPPER ${OUTPUTCONFIG} OUTPUTCONFIG)
    set_target_properties(DCUProvider PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY_${OUTPUTCONFIG} "${CMAKE_BINARY_DIR}/output"
        LIBRARY_OUTPUT_DIRECTORY_${OUTPUTCONFIG} "${CMAKE_BINARY_DIR}/output"
    )
endforeach()
```

- [ ] **Step 3: Verify the macOS build is unaffected (no-op check)**

If you have access to a macOS machine or CI for this repo, run `./build-macos.sh` and confirm `build-macos/output/mac.xpl` is produced exactly as before. If no macOS machine is available in this session, skip this step — the loop is a no-op for single-config generators by construction (`CMAKE_CONFIGURATION_TYPES` is empty for Makefiles), so this is a correctness argument, not something this environment can execute. Do not skip it if a macOS machine is available later — flag it in the follow-up report.

- [ ] **Step 4: Commit**

```bash
git add DCUProviderPlugin/CMakeLists.txt
git commit -m "$(cat <<'EOF'
fix: flatten CMake output directory for multi-config generators

Visual Studio's generator appends a Release/Debug subfolder to
LIBRARY_OUTPUT_DIRECTORY unless the per-config property variants are
also pinned. Needed so the new native build-windows.sh (MSVC) finds
its .xpl at output/win.xpl like every other build script expects.
EOF
)"
```

---

### Task 3: New native `build-windows.sh`

**Files:**
- Create: `DCUProviderPlugin/build-windows.sh`

**Interfaces:**
- Consumes: `DCUProviderPlugin/CMakeLists.txt`'s WIN32/MSVC branch (already links `XPLM_64.lib` directly, no changes needed there) and the flattened output dir from Task 2.
- Produces: `build-windows/output/win.xpl`, referenced by Task 4's docs.

- [ ] **Step 1: Create the script**

Write `DCUProviderPlugin/build-windows.sh`:

```bash
#!/bin/bash
# Native build for Windows (MSVC via Visual Studio Build Tools) - no cross-compiling

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${YELLOW}╔════════════════════════════════════════╗${NC}"
echo -e "${YELLOW}║   DCU Provider - Windows Build (native)║${NC}"
echo -e "${YELLOW}╚════════════════════════════════════════╝${NC}\n"

# Check if debug mode is requested
BUILD_TYPE="Release"
if [ "$1" == "debug" ]; then
    BUILD_TYPE="Debug"
    echo -e "${YELLOW}Debug build${NC}\n"
fi

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Build directory
BUILD_DIR="${SCRIPT_DIR}/build-windows"

# Output directory
OUTPUT_DIR="${BUILD_DIR}/output"

# ============ Check Prerequisites ============

echo -e "${BLUE}Checking prerequisites...${NC}"

if ! command -v cmake &> /dev/null; then
    echo -e "${RED}✗ CMake not installed${NC}"
    echo -e "${YELLOW}Install with: winget install Kitware.CMake${NC}"
    exit 1
fi
echo -e "${GREEN}✓ CMake: $(cmake --version | head -1)${NC}"

VSWHERE="/c/Program Files (x86)/Microsoft Visual Studio/Installer/vswhere.exe"
if [ ! -f "$VSWHERE" ]; then
    echo -e "${RED}✗ Visual Studio Build Tools not found${NC}"
    echo -e "${YELLOW}Install with: winget install --id Microsoft.VisualStudio.2022.BuildTools --override \"--quiet --add Microsoft.VisualStudio.Workload.VCTools --includeRecommended\"${NC}"
    echo -e "${YELLOW}Or download: https://visualstudio.microsoft.com/downloads/#build-tools-for-visual-studio-2022${NC}"
    echo -e "${YELLOW}(select the \"Desktop development with C++\" workload)${NC}"
    exit 1
fi

VS_INSTALL_PATH=$("$VSWHERE" -latest -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath)
if [ -z "$VS_INSTALL_PATH" ]; then
    echo -e "${RED}✗ Visual Studio C++ workload not found${NC}"
    echo -e "${YELLOW}Install with: winget install --id Microsoft.VisualStudio.2022.BuildTools --override \"--quiet --add Microsoft.VisualStudio.Workload.VCTools --includeRecommended\"${NC}"
    echo -e "${YELLOW}Or open Visual Studio Installer and add the \"Desktop development with C++\" workload${NC}"
    exit 1
fi
echo -e "${GREEN}✓ Visual Studio C++ toolchain: ${VS_INSTALL_PATH}${NC}"

# ============ Clean Build ============

echo -e "\n${BLUE}Cleaning previous build...${NC}"
rm -rf "${BUILD_DIR}"
mkdir -p "${BUILD_DIR}"

# ============ Configure with CMake ============

echo -e "\n${BLUE}Configuring with CMake for Windows (native)...${NC}"
cd "${BUILD_DIR}"

cmake .. \
    -A x64 \
    -DXPLANE_SDK="${SCRIPT_DIR}/SDK"

if [ $? -ne 0 ]; then
    echo -e "${RED}✗ CMake configuration failed${NC}"
    exit 1
fi
echo -e "${GREEN}✓ CMake configuration successful${NC}"

# ============ Build ============

echo -e "\n${BLUE}Building plugin for Windows (${BUILD_TYPE})...${NC}"
cmake --build . --config "${BUILD_TYPE}" --parallel

if [ $? -ne 0 ]; then
    echo -e "${RED}✗ Build failed${NC}"
    exit 1
fi

# ============ Verify Output ============

if [ -f "${OUTPUT_DIR}/win.xpl" ]; then
    echo -e "\n${GREEN}╔════════════════════════════════════════╗${NC}"
    echo -e "${GREEN}║          BUILD SUCCESSFUL!             ║${NC}"
    echo -e "${GREEN}╚════════════════════════════════════════╝${NC}"
    echo -e "\n${GREEN}Plugin built: ${OUTPUT_DIR}/win.xpl${NC}"

    FILE_SIZE=$(ls -lh "${OUTPUT_DIR}/win.xpl" | awk '{print $5}')
    echo -e "${BLUE}File size: ${FILE_SIZE}${NC}"

    echo -e "\n${YELLOW}To install:${NC}"
    echo -e "1. Place it in: X-Plane 12/Resources/plugins/DCUProvider/64/win.xpl"
else
    echo -e "${RED}✗ Build failed - plugin file not found${NC}"
    exit 1
fi
```

- [ ] **Step 2: Make it executable and syntax-check it**

Run: `chmod +x DCUProviderPlugin/build-windows.sh && bash -n DCUProviderPlugin/build-windows.sh && echo OK`
Expected: `OK`

- [ ] **Step 3: Run it and confirm the prereq-check failure path works**

This dev machine currently has neither `cmake` nor Visual Studio Build Tools installed. Run:

`cd DCUProviderPlugin && ./build-windows.sh; echo "exit code: $?"`

Expected: banner prints, then a red `✗ CMake not installed` line (or, if `cmake` gets installed before this step runs, a red `✗ Visual Studio Build Tools not found` / `✗ Visual Studio C++ workload not found` line instead), followed by install guidance and `exit code: 1`. This confirms the script fails closed with a helpful message instead of a raw CMake error — it does not confirm a successful build, which needs a machine with VS Build Tools installed (see Task 3 follow-up note below).

- [ ] **Step 4: Commit**

```bash
git add DCUProviderPlugin/build-windows.sh
git commit -m "$(cat <<'EOF'
feat: add native Windows build script (MSVC, no cross-compiling)

Builds the plugin directly on a Windows machine via Visual Studio
Build Tools, mirroring build-macos.sh's structure. Checks for cmake
and the VS C++ workload (via vswhere) up front and prints install
guidance instead of failing on a raw CMake error.
EOF
)"
```

**Follow-up note (not a task step, just flag it in your final report):** once a Windows machine with VS Build Tools is available, run `./build-windows.sh` and `./build-windows.sh debug` there to confirm the success path (`build-windows/output/win.xpl` produced, correct size/architecture) — this plan's execution environment can't do that itself.

---

### Task 4: Update docs

**Files:**
- Modify: `CLAUDE.md:45` (repo root)
- Modify: `DCUProviderPlugin/CLAUDE.md` (Commands section, `./build-windows.sh` line)
- Modify: `DCUProviderPlugin/README.md:14-18`

**Interfaces:**
- None — pure documentation, no code interfaces produced or consumed.

- [ ] **Step 1: Update repo-root `CLAUDE.md`**

In `CLAUDE.md`, replace:

```
./build-windows.sh        # cross-compile with MinGW
```

with:

```
./build-windows.sh        # native build on Windows (MSVC)
./build-xc-windows.sh     # cross-compile for Windows from macOS (MinGW)
```

- [ ] **Step 2: Update `DCUProviderPlugin/CLAUDE.md`**

Find the line:

```
./build-windows.sh        # Cross-compile with MinGW (needs toolchain-mingw64.cmake)
```

Replace with:

```
./build-windows.sh        # Native build on Windows (MSVC, needs VS Build Tools)
./build-xc-windows.sh     # Cross-compile for Windows from macOS (MinGW, needs toolchain-mingw64.cmake)
```

- [ ] **Step 3: Update `DCUProviderPlugin/README.md`**

Replace:

```markdown
### Windows (Cross-Compile mit MinGW)

```bash
./build-windows.sh
```
```

with:

```markdown
### Windows (nativ, MSVC)

```bash
./build-windows.sh          # Release Build
./build-windows.sh debug    # Debug Build
```

### Windows (Cross-Compile mit MinGW, von macOS aus)

```bash
./build-xc-windows.sh
```
```

- [ ] **Step 4: Verify no other stale references to the old filename remain**

Run: `grep -rn "build-windows.sh" CLAUDE.md DCUProviderPlugin/CLAUDE.md DCUProviderPlugin/README.md DCUProviderPlugin/build-all.sh`
Expected: every match is either the new native `build-windows.sh` (docs Steps 1-3, and Task 1's rewired `build-all.sh` should show none) — no line should say "cross-compile"/"MinGW" next to `build-windows.sh` anymore; those now say `build-xc-windows.sh`.

- [ ] **Step 5: Commit**

```bash
git add CLAUDE.md DCUProviderPlugin/CLAUDE.md DCUProviderPlugin/README.md
git commit -m "$(cat <<'EOF'
docs: document native build-windows.sh and renamed build-xc-windows.sh
EOF
)"
```
