#!/bin/bash

# ROS 2 Path Emitter Launcher
# Starts the C++ gRPC path planning service

set -e

echo "================================================"
echo "📡 Starting Univarm ROS 2 Path Emitter (gRPC)"
echo "================================================"
echo ""

# Color codes
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m'

# Configuration
PLANNER_ADDR="${PLANNER_ADDR:-127.0.0.1:50051}"

# Check ROS 2
if [ -z "$ROS_DISTRO" ]; then
    for distro in humble iron jazzy rolling; do
        if [ -f "/opt/ros/$distro/setup.bash" ]; then
            export ROS_DISTRO=$distro
            break
        fi
    done
fi

if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}❌ ROS 2 not found${NC}"
    exit 1
fi

echo -e "${GREEN}✓ Using ROS 2 $ROS_DISTRO${NC}"

# Source ROS 2
source /opt/ros/$ROS_DISTRO/setup.bash

# Check if workspace is built
if [ ! -f "ros2_ws/install/setup.bash" ]; then
    echo -e "${YELLOW}⚠ Workspace not built. Running build...${NC}"
    ./build-ros2.sh
fi

# Source workspace
echo -e "${BLUE}Sourcing workspace...${NC}"
source ros2_ws/install/setup.bash

echo ""
echo -e "${BLUE}Configuration:${NC}"
echo "  Planner address: $PLANNER_ADDR"
echo ""

# Start emitter
echo -e "${BLUE}Starting path emitter...${NC}"
echo ""

ros2 run univarm_path_emitter_cpp path_emitter \
  --ros-args -p planner_addr:=$PLANNER_ADDR

echo ""
echo -e "${GREEN}✓ Path emitter stopped${NC}"

