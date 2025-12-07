#!/bin/bash

# ROS 2 Workspace Build Script
# Builds all Univarm ROS 2 packages

set -e

echo "================================================"
echo "🤖 Building Univarm ROS 2 Workspace"
echo "================================================"
echo ""

# Color codes
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m'

# Check ROS 2 installation
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${YELLOW}⚠ ROS_DISTRO not set. Detecting...${NC}"
    
    # Try common ROS 2 distros
    for distro in humble iron jazzy rolling; do
        if [ -f "/opt/ros/$distro/setup.bash" ]; then
            export ROS_DISTRO=$distro
            echo -e "${GREEN}✓ Found ROS 2 $ROS_DISTRO${NC}"
            break
        fi
    done
    
    if [ -z "$ROS_DISTRO" ]; then
        echo -e "${RED}❌ ROS 2 not found. Please install ROS 2 Humble or newer.${NC}"
        echo "   See: https://docs.ros.org/en/humble/Installation.html"
        exit 1
    fi
fi

echo -e "${GREEN}✓ Using ROS 2 $ROS_DISTRO${NC}"
echo ""

# Source ROS 2
echo -e "${BLUE}Sourcing ROS 2 environment...${NC}"
source /opt/ros/$ROS_DISTRO/setup.bash

# Navigate to workspace
cd "/Users/frankvanlaarhoven/Downloads/NAVΛ STUDIO IDE/ros2_ws"

# Check for rosdep
if ! command -v rosdep &> /dev/null; then
    echo -e "${RED}❌ rosdep not found${NC}"
    echo "   Install: sudo apt-get install python3-rosdep"
    exit 1
fi

# Install dependencies
echo -e "${BLUE}Installing dependencies with rosdep...${NC}"
rosdep install --from-paths src --ignore-src -y || echo -e "${YELLOW}⚠ Some dependencies may be missing${NC}"

echo ""
echo -e "${BLUE}Building packages with colcon...${NC}"
echo "This may take a few minutes..."
echo ""

# Build with colcon
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

if [ $? -eq 0 ]; then
    echo ""
    echo "================================================"
    echo -e "${GREEN}✅ Build Successful!${NC}"
    echo "================================================"
    echo ""
    echo "Packages built:"
    echo "  • helix_figure3 - Humanoid robot skeleton"
    echo "  • univarm_drive_bridge_py - Python backend bridge"
    echo "  • univarm_kobuki_bringup - Kobuki robot launcher"
    echo "  • univarm_msgs - Custom ROS 2 messages"
    echo "  • univarm_path_emitter_cpp - gRPC path emitter"
    echo "  • univarm_tb2_bringup - TurtleBot2 launcher"
    echo ""
    echo "To use:"
    echo "  source install/setup.bash"
    echo ""
    echo "Launch Kobuki demo:"
    echo "  ros2 launch univarm_kobuki_bringup bringup.launch.py use_rviz:=true"
    echo ""
    echo "Or use the quick start script:"
    echo "  ./start-kobuki-demo.sh"
    echo ""
else
    echo ""
    echo -e "${RED}❌ Build failed${NC}"
    echo "Check the output above for errors"
    exit 1
fi

