#!/bin/bash

# Stop ROBOTIS-SYSTEMIC Integration Services

set -e

echo "================================================"
echo "🛑 Stopping ROBOTIS-SYSTEMIC Integration"
echo "================================================"
echo ""

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

NAVA_DIR="/Users/frankvanlaarhoven/Downloads/NAVΛ STUDIO IDE"

# Function to stop a service by PID file
stop_service() {
    local pid_file=$1
    local name=$2
    
    if [ -f "$pid_file" ]; then
        local pid=$(cat "$pid_file")
        if ps -p $pid > /dev/null 2>&1; then
            echo -e "${YELLOW}Stopping $name (PID: $pid)...${NC}"
            kill $pid 2>/dev/null || true
            sleep 1
            if ps -p $pid > /dev/null 2>&1; then
                echo -e "${YELLOW}Force killing $name...${NC}"
                kill -9 $pid 2>/dev/null || true
            fi
            echo -e "${GREEN}✓ $name stopped${NC}"
        else
            echo -e "${YELLOW}⚠ $name process not found${NC}"
        fi
        rm -f "$pid_file"
    else
        echo -e "${YELLOW}⚠ No PID file found for $name${NC}"
    fi
}

# Stop services
stop_service "$NAVA_DIR/.nava.pid" "NAVΛ Studio IDE"
stop_service "$NAVA_DIR/.univarm.pid" "Univarm Web"
stop_service "$NAVA_DIR/.robotd.pid" "robotd"

# Kill any remaining processes on the ports
echo ""
echo "Cleaning up ports..."
for port in 8080 3000 5175; do
    if lsof -Pi :$port -sTCP:LISTEN -t >/dev/null 2>&1; then
        echo -e "${YELLOW}Killing process on port $port...${NC}"
        lsof -ti:$port | xargs kill -9 2>/dev/null || true
    fi
done

echo ""
echo -e "${GREEN}✅ All services stopped${NC}"
echo ""

