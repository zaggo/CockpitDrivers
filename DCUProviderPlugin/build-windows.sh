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
    -DXPLANE_SDK="${SCRIPT_DIR}/../XPlaneSDK"

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

    # ============ Auto-Install Plugin ============
    INSTALL_PATH="/x/X-Plane 12/Resources/plugins/DCUProvider/64/win.xpl"
    mkdir -p "$(dirname "$INSTALL_PATH")"
    cp "${OUTPUT_DIR}/win.xpl" "$INSTALL_PATH"
    echo -e "\n${GREEN}✓ Plugin automatisch installiert nach:${NC} ${BLUE}$INSTALL_PATH${NC}"
else
    echo -e "${RED}✗ Build failed - plugin file not found${NC}"
    exit 1
fi
