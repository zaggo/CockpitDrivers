#!/bin/bash
# Build Motion Provider Plugin for all platforms (macOS and Windows)

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

echo -e "${CYAN}╔════════════════════════════════════════╗${NC}"
echo -e "${CYAN}║   Motion Provider - Multi-Platform Build  ║${NC}"
echo -e "${CYAN}╚════════════════════════════════════════╝${NC}\n"

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Output directory for fat plugin
BUILD_ALL_DIR="${SCRIPT_DIR}/build-all"
FAT_PLUGIN_DIR="${BUILD_ALL_DIR}/MotionProvider"
PLUGIN_64_DIR="${FAT_PLUGIN_DIR}/64"

# ============ Clean previous fat plugin ============

echo -e "${BLUE}Cleaning previous builds...${NC}"
rm -rf "${BUILD_ALL_DIR}"
mkdir -p "${PLUGIN_64_DIR}"
echo -e "${GREEN}✓ Output directories created${NC}"

# ============ Build macOS ============

echo -e "\n${YELLOW}════════════════════════════════════════${NC}"
echo -e "${YELLOW}Building for macOS...${NC}"
echo -e "${YELLOW}════════════════════════════════════════${NC}\n"

if [ -f "${SCRIPT_DIR}/build-macos.sh" ]; then
    BUILD_ALL_MODE=1 "${SCRIPT_DIR}/build-macos.sh"
    
    if [ -f "${SCRIPT_DIR}/build-macos/output/mac.xpl" ]; then
        cp "${SCRIPT_DIR}/build-macos/output/mac.xpl" "${PLUGIN_64_DIR}/mac.xpl"
        echo -e "${GREEN}✓ macOS plugin copied to ${PLUGIN_64_DIR}/mac.xpl${NC}"
    else
        echo -e "${RED}✗ macOS build failed - plugin not found${NC}"
        exit 1
    fi
else
    echo -e "${RED}✗ build-macos.sh not found${NC}"
    exit 1
fi

# ============ Build Windows ============

echo -e "\n${YELLOW}════════════════════════════════════════${NC}"
echo -e "${YELLOW}Building for Windows...${NC}"
echo -e "${YELLOW}════════════════════════════════════════${NC}\n"

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

# ============ Create README ============

cat > "${FAT_PLUGIN_DIR}/README.txt" << 'EOF'
Motion Provider Plugin for X-Plane 12
===================================

Installation:
-------------
Copy the entire "MotionProvider" folder to:
    X-Plane 12/Resources/plugins/

The plugin will automatically load the correct version for your platform:
- 64/mac.xpl for macOS
- 64/win.xpl for Windows

The plugin provides a status window accessible via:
    Plugins > Motion Provider > Show Status Window

Build Information:
------------------
EOF

# Add build date and file sizes
echo "Built: $(date)" >> "${FAT_PLUGIN_DIR}/README.txt"
echo "" >> "${FAT_PLUGIN_DIR}/README.txt"
echo "macOS plugin size: $(ls -lh ${PLUGIN_64_DIR}/mac.xpl | awk '{print $5}')" >> "${FAT_PLUGIN_DIR}/README.txt"
echo "Windows plugin size: $(ls -lh ${PLUGIN_64_DIR}/win.xpl | awk '{print $5}')" >> "${FAT_PLUGIN_DIR}/README.txt"

# ============ Summary ============

echo -e "\n${GREEN}╔════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║      MULTI-PLATFORM BUILD SUCCESS!     ║${NC}"
echo -e "${GREEN}╚════════════════════════════════════════╝${NC}\n"

echo -e "${CYAN}Plugin structure:${NC}"
tree -L 2 "${FAT_PLUGIN_DIR}" 2>/dev/null || find "${FAT_PLUGIN_DIR}" -type f -o -type d | sed 's|[^/]*/| |g'

echo -e "\n${BLUE}File sizes:${NC}"
echo -e "  macOS:   $(ls -lh ${PLUGIN_64_DIR}/mac.xpl | awk '{print $5}')"
echo -e "  Windows: $(ls -lh ${PLUGIN_64_DIR}/win.xpl | awk '{print $5}')"

echo -e "\n${YELLOW}Installation:${NC}"
echo -e "Copy the entire folder to X-Plane:"
echo -e "  ${CYAN}${FAT_PLUGIN_DIR}${NC}"
echo -e "  → ${CYAN}X-Plane 12/Resources/plugins/${NC}"

echo -e "\n${GREEN}✓ Multi-platform plugin ready!${NC}\n"
