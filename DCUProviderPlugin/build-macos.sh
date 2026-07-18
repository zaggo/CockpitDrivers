#!/bin/bash
# filepath: /Users/zaggo/Developer/CockpitDrivers/DCUProviderPlugin/build.sh

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Check if debug mode is requested
BUILD_TYPE="Release"
if [ "$1" == "debug" ]; then
    BUILD_TYPE="Debug"
    echo -e "${YELLOW}╔════════════════════════════════════════╗${NC}"
    echo -e "${YELLOW}║   DCU Plugin - DEBUG Build             ║${NC}"
    echo -e "${YELLOW}╚════════════════════════════════════════╝${NC}\n"
else
    echo -e "${YELLOW}╔════════════════════════════════════════╗${NC}"
    echo -e "${YELLOW}║   DCU Provider Plugin - Build Script   ║${NC}"
    echo -e "${YELLOW}╚════════════════════════════════════════╝${NC}\n"
fi

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Build directory
if [ "$BUILD_TYPE" == "Debug" ]; then
    BUILD_DIR="${SCRIPT_DIR}/build-debug"
else
    BUILD_DIR="${SCRIPT_DIR}/build-macos"
fi

# Output directory
OUTPUT_DIR="${BUILD_DIR}/output"


# X-Plane plugins directory (customized for user)
XPLANE_PLUGINS_DIR="/Volumes/1TBSSD/XPlane/X-Plane 12/Resources/plugins/DCUProvider/mac_x64"

# ============ Check Prerequisites ============

echo -e "${BLUE}Checking prerequisites...${NC}"

if ! command -v cmake &> /dev/null; then
    echo -e "${RED}✗ CMake not installed${NC}"
    echo -e "${YELLOW}Install with: brew install cmake${NC}"
    exit 1
fi
echo -e "${GREEN}✓ CMake: $(cmake --version | head -1)${NC}"

if ! command -v make &> /dev/null; then
    echo -e "${RED}✗ Make not installed${NC}"
    exit 1
fi
echo -e "${GREEN}✓ Make installed${NC}"

if ! command -v clang &> /dev/null; then
    echo -e "${RED}✗ Clang not installed${NC}"
    echo -e "${YELLOW}Install Xcode Command Line Tools: xcode-select --install${NC}"
    exit 1
fi
echo -e "${GREEN}✓ Clang: $(clang --version | head -1)${NC}\n"

# ============ Check SDK ============

echo -e "${BLUE}Checking X-Plane SDK...${NC}"

if [ -d "${SCRIPT_DIR}/SDK" ]; then
    echo -e "${GREEN}✓ Found local SDK at: ${SCRIPT_DIR}/SDK${NC}"
    XPLANE_SDK="${SCRIPT_DIR}/SDK"
elif [ -n "$XPLANE_SDK" ] && [ -d "$XPLANE_SDK" ]; then
    echo -e "${GREEN}✓ Using XPLANE_SDK: ${XPLANE_SDK}${NC}"
else
    echo -e "${RED}✗ X-Plane SDK not found${NC}"
    echo -e "${YELLOW}Place SDK folder at: ${SCRIPT_DIR}/SDK${NC}"
    echo -e "${YELLOW}Or set: export XPLANE_SDK=/path/to/sdk${NC}"
    exit 1
fi

echo ""

# ============ Create Build Directory ============

if [ ! -d "$BUILD_DIR" ]; then
    mkdir -p "$BUILD_DIR"
    echo -e "${GREEN}✓ Created build directory${NC}"
fi

# ============ Run CMake ============

echo -e "${BLUE}Running CMake...${NC}\n"

cd "$BUILD_DIR"

cmake \
    -DXPLANE_SDK="${XPLANE_SDK}" \
    -DCMAKE_BUILD_TYPE=${BUILD_TYPE} \
    -DXPLANE_PLUGINS_DIR="${XPLANE_PLUGINS_DIR}" \
    -DCMAKE_C_COMPILER=clang \
    -DCMAKE_CXX_COMPILER=clang++ \
    -DXPLM200=1 \
    -DXPLM210=1 \
    -DXPLM300=1 \
    -DXPLM301=1 \
    -DXPLM400=1 \
    ..

if [ $? -ne 0 ]; then
    echo -e "\n${RED}✗ CMake failed${NC}"
    exit 1
fi

echo -e "${BLUE}Build Type: ${BUILD_TYPE}${NC}"

echo ""

# ============ Build ============

echo -e "${BLUE}Building plugin...${NC}"

NUM_CORES=$(sysctl -n hw.ncpu)
echo -e "${YELLOW}Using ${NUM_CORES} CPU cores\n${NC}"

make -j$NUM_CORES

if [ $? -ne 0 ]; then
    echo -e "\n${RED}✗ Build failed${NC}"
    exit 1
fi

echo ""

# ============ Verify Build Output ============

if [ ! -f "${OUTPUT_DIR}/mac.xpl" ]; then
    echo -e "${RED}✗ Plugin binary not found${NC}"
    exit 1
fi

# ============ Display Results ============

echo -e "${YELLOW}╔════════════════════════════════════════╗${NC}"
echo -e "${YELLOW}║   BUILD SUCCESSFUL                     ║${NC}"
echo -e "${YELLOW}╚════════════════════════════════════════╝${NC}\n"


echo -e "${GREEN}Plugin Details:${NC}"
ls -lh "${OUTPUT_DIR}/mac.xpl"

echo -e "\n${GREEN}Plugin Location:${NC}"
echo -e "${BLUE}  ${OUTPUT_DIR}/mac.xpl${NC}"

# ============ Auto-Install Plugin ============
INSTALL_PATH="/Volumes/1TBSSD/XPlane/X-Plane 12/Resources/plugins/DCUProvider/64/mac.xpl"
mkdir -p "$(dirname "$INSTALL_PATH")"
cp "${OUTPUT_DIR}/mac.xpl" "$INSTALL_PATH"
echo -e "${GREEN}✓ Plugin automatisch installiert nach:${NC} ${BLUE}$INSTALL_PATH${NC}"

# ============ Installation ============

if [ -d "$XPLANE_PLUGINS_DIR" ] && [ -z "$BUILD_ALL_MODE" ]; then
    echo -e "\n${YELLOW}X-Plane plugins directory found${NC}"
    
    read -p "$(echo -e ${YELLOW}Install plugin? [y/n]${NC} )" -n 1 -r
    echo
    
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        cp "${OUTPUT_DIR}/mac.xpl" "${XPLANE_PLUGINS_DIR}/"
        echo -e "${GREEN}✓ Plugin installed${NC}"
        echo -e "${GREEN}✓ Restart X-Plane to load${NC}"
    fi
elif [ -z "$BUILD_ALL_MODE" ]; then
    echo -e "\n${YELLOW}Manual installation:${NC}"
    echo -e "${BLUE}  cp ${OUTPUT_DIR}/mac.xpl /Volumes/1TBSSD/XPlane/X-Plane\ 12/Resources/plugins/DCUProvider/64/mac.xpl${NC}"
fi

echo ""

cd "$SCRIPT_DIR"