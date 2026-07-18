#!/bin/bash
# Cross-compile for Windows from macOS

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${YELLOW}╔════════════════════════════════════════╗${NC}"
echo -e "${YELLOW}║   DCU Provider - Windows Build         ║${NC}"
echo -e "${YELLOW}╚════════════════════════════════════════╝${NC}\n"

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
    echo -e "${YELLOW}Install with: brew install cmake${NC}"
    exit 1
fi
echo -e "${GREEN}✓ CMake: $(cmake --version | head -1)${NC}"

if ! command -v x86_64-w64-mingw32-gcc &> /dev/null; then
    echo -e "${RED}✗ MinGW-w64 not installed${NC}"
    echo -e "${YELLOW}Install with: brew install mingw-w64${NC}"
    exit 1
fi
echo -e "${GREEN}✓ MinGW-w64: $(x86_64-w64-mingw32-gcc --version | head -1)${NC}"

# ============ Clean Build ============

echo -e "\n${BLUE}Cleaning previous build...${NC}"
rm -rf "${BUILD_DIR}"
mkdir -p "${BUILD_DIR}"

# ============ Configure with CMake ============

echo -e "\n${BLUE}Configuring with CMake for Windows...${NC}"
cd "${BUILD_DIR}"

cmake .. \
    -DCMAKE_TOOLCHAIN_FILE="${SCRIPT_DIR}/toolchain-mingw64.cmake" \
    -DCMAKE_BUILD_TYPE=Release \
    -DXPLANE_SDK="${SCRIPT_DIR}/SDK"

if [ $? -ne 0 ]; then
    echo -e "${RED}✗ CMake configuration failed${NC}"
    exit 1
fi
echo -e "${GREEN}✓ CMake configuration successful${NC}"

# ============ Build ============

echo -e "\n${BLUE}Building plugin for Windows...${NC}"
make -j$(sysctl -n hw.ncpu)

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
    
    echo -e "\n${BLUE}File info:${NC}"
    file "${OUTPUT_DIR}/win.xpl"
    
    echo -e "\n${YELLOW}To use this plugin on Windows:${NC}"
    echo -e "1. Copy ${OUTPUT_DIR}/win.xpl to Windows"
    echo -e "2. Place it in: X-Plane 12/Resources/plugins/DCUProvider/64/"
else
    echo -e "${RED}✗ Build failed - plugin file not found${NC}"
    exit 1
fi
