#!/bin/bash

# ROBOTIS-SYSTEMIC Integration Verification Script
# Verifies that all components are properly integrated and running

set -e

echo "================================================"
echo "🔍 ROBOTIS-SYSTEMIC Integration Verification"
echo "================================================"
echo ""

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# Configuration
ROBOTIS_DIR="/Users/frankvanlaarhoven/Downloads/ROBOTIS-SYSTEMIC"
NAVA_DIR="/Users/frankvanlaarhoven/Downloads/NAVΛ STUDIO IDE"

# Counters
PASSED=0
FAILED=0
WARNINGS=0

# Test result function
test_result() {
    local test_name=$1
    local result=$2
    local message=$3
    
    if [ "$result" = "PASS" ]; then
        echo -e "${GREEN}✓ PASS${NC}: $test_name"
        [ -n "$message" ] && echo -e "  ${BLUE}ℹ${NC} $message"
        PASSED=$((PASSED + 1))
    elif [ "$result" = "FAIL" ]; then
        echo -e "${RED}✗ FAIL${NC}: $test_name"
        [ -n "$message" ] && echo -e "  ${RED}!${NC} $message"
        FAILED=$((FAILED + 1))
    elif [ "$result" = "WARN" ]; then
        echo -e "${YELLOW}⚠ WARN${NC}: $test_name"
        [ -n "$message" ] && echo -e "  ${YELLOW}!${NC} $message"
        WARNINGS=$((WARNINGS + 1))
    fi
}

echo "1. Directory Structure Verification"
echo "───────────────────────────────────────────────"

# Check ROBOTIS-SYSTEMIC directory
if [ -d "$ROBOTIS_DIR" ]; then
    test_result "ROBOTIS-SYSTEMIC directory" "PASS" "$ROBOTIS_DIR"
else
    test_result "ROBOTIS-SYSTEMIC directory" "FAIL" "Directory not found: $ROBOTIS_DIR"
fi

# Check NAVΛ Studio IDE directory
if [ -d "$NAVA_DIR" ]; then
    test_result "NAVΛ Studio IDE directory" "PASS" "$NAVA_DIR"
else
    test_result "NAVΛ Studio IDE directory" "FAIL" "Directory not found: $NAVA_DIR"
fi

# Check robotd service
if [ -f "$ROBOTIS_DIR/services/robotd/Cargo.toml" ]; then
    test_result "robotd service files" "PASS" "Rust backend found"
else
    test_result "robotd service files" "FAIL" "robotd Cargo.toml not found"
fi

# Check Univarm Web app
if [ -f "$ROBOTIS_DIR/apps/web/package.json" ]; then
    test_result "Univarm Web files" "PASS" "Next.js frontend found"
else
    test_result "Univarm Web files" "FAIL" "Univarm Web package.json not found"
fi

echo ""
echo "2. Integration Files Verification"
echo "───────────────────────────────────────────────"

# Check integration files
INTEGRATION_FILES=(
    "$NAVA_DIR/public/univarm/univarm-overlay.js"
    "$NAVA_DIR/public/univarm/univarm-overlay.css"
    "$NAVA_DIR/public/univarm/register-univarm.js"
    "$NAVA_DIR/public/univarm/apps.json"
    "$NAVA_DIR/public/univarm/icon.svg"
    "$NAVA_DIR/src/apps/UnivarmApp.tsx"
)

for file in "${INTEGRATION_FILES[@]}"; do
    filename=$(basename "$file")
    if [ -f "$file" ]; then
        test_result "Integration file: $filename" "PASS"
    else
        test_result "Integration file: $filename" "FAIL" "File not found"
    fi
done

# Check startup scripts
if [ -f "$NAVA_DIR/start-robotis-system.sh" ]; then
    test_result "Startup script" "PASS"
    if [ -x "$NAVA_DIR/start-robotis-system.sh" ]; then
        test_result "Startup script executable" "PASS"
    else
        test_result "Startup script executable" "WARN" "Run: chmod +x start-robotis-system.sh"
    fi
else
    test_result "Startup script" "FAIL" "start-robotis-system.sh not found"
fi

echo ""
echo "3. Configuration Files Verification"
echo "───────────────────────────────────────────────"

# Check vite.config.ts for proxy settings
if [ -f "$NAVA_DIR/vite.config.ts" ]; then
    if grep -q "/univarm" "$NAVA_DIR/vite.config.ts" && grep -q "'/api'" "$NAVA_DIR/vite.config.ts"; then
        test_result "Vite proxy configuration" "PASS" "Univarm and API proxies configured"
    else
        test_result "Vite proxy configuration" "WARN" "Proxy configuration may be incomplete"
    fi
else
    test_result "Vite config file" "FAIL" "vite.config.ts not found"
fi

# Check environment files
if [ -f "$ROBOTIS_DIR/apps/web/.env.local" ]; then
    test_result "Univarm environment file" "PASS" ".env.local exists"
else
    test_result "Univarm environment file" "WARN" ".env.local will be created on startup"
fi

if [ -f "$NAVA_DIR/.env.robotis" ]; then
    test_result "NAVΛ integration environment" "PASS" ".env.robotis exists"
else
    test_result "NAVΛ integration environment" "WARN" ".env.robotis recommended"
fi

echo ""
echo "4. Dependencies Verification"
echo "───────────────────────────────────────────────"

# Check Node.js
if command -v node &> /dev/null; then
    NODE_VERSION=$(node --version)
    test_result "Node.js installation" "PASS" "Version: $NODE_VERSION"
else
    test_result "Node.js installation" "FAIL" "Node.js not found"
fi

# Check npm
if command -v npm &> /dev/null; then
    NPM_VERSION=$(npm --version)
    test_result "npm installation" "PASS" "Version: $NPM_VERSION"
else
    test_result "npm installation" "FAIL" "npm not found"
fi

# Check Rust
if command -v rustc &> /dev/null; then
    RUST_VERSION=$(rustc --version)
    test_result "Rust installation" "PASS" "Version: $RUST_VERSION"
else
    test_result "Rust installation" "FAIL" "Rust not found - required for robotd"
fi

# Check Cargo
if command -v cargo &> /dev/null; then
    CARGO_VERSION=$(cargo --version)
    test_result "Cargo installation" "PASS" "Version: $CARGO_VERSION"
else
    test_result "Cargo installation" "FAIL" "Cargo not found - required for robotd"
fi

# Check NAVΛ node_modules
if [ -d "$NAVA_DIR/node_modules" ]; then
    test_result "NAVΛ dependencies" "PASS" "node_modules exists"
else
    test_result "NAVΛ dependencies" "WARN" "Run: npm install in NAVΛ directory"
fi

# Check Univarm node_modules
if [ -d "$ROBOTIS_DIR/apps/web/node_modules" ]; then
    test_result "Univarm dependencies" "PASS" "node_modules exists"
else
    test_result "Univarm dependencies" "WARN" "Run: npm install in Univarm directory"
fi

echo ""
echo "5. Service Status Verification"
echo "───────────────────────────────────────────────"

# Check if services are running
check_service() {
    local port=$1
    local name=$2
    
    if lsof -Pi :$port -sTCP:LISTEN -t >/dev/null 2>&1; then
        test_result "$name (port $port)" "PASS" "Service is running"
        return 0
    else
        test_result "$name (port $port)" "WARN" "Service not running"
        return 1
    fi
}

SERVICES_RUNNING=0
check_service 8080 "robotd" && SERVICES_RUNNING=$((SERVICES_RUNNING + 1))
check_service 3000 "Univarm Web" && SERVICES_RUNNING=$((SERVICES_RUNNING + 1))
check_service 5175 "NAVΛ Studio IDE" && SERVICES_RUNNING=$((SERVICES_RUNNING + 1))

echo ""
echo "6. API Endpoint Verification"
echo "───────────────────────────────────────────────"

# Only test if services are running
if [ $SERVICES_RUNNING -gt 0 ]; then
    # Test robotd API (if running)
    if lsof -Pi :8080 -sTCP:LISTEN -t >/dev/null 2>&1; then
        if curl -s -f http://localhost:8080/api/rt/subscribe?token=demo-token-123 -m 2 >/dev/null 2>&1; then
            test_result "robotd API endpoint" "PASS" "http://localhost:8080/api accessible"
        else
            test_result "robotd API endpoint" "WARN" "API may not be fully ready"
        fi
    fi
    
    # Test Univarm Web (if running)
    if lsof -Pi :3000 -sTCP:LISTEN -t >/dev/null 2>&1; then
        if curl -s -f http://localhost:3000 -m 2 >/dev/null 2>&1; then
            test_result "Univarm Web frontend" "PASS" "http://localhost:3000 accessible"
        else
            test_result "Univarm Web frontend" "WARN" "Frontend may not be fully ready"
        fi
    fi
    
    # Test NAVΛ Studio (if running)
    if lsof -Pi :5175 -sTCP:LISTEN -t >/dev/null 2>&1; then
        if curl -s -f http://localhost:5175 -m 2 >/dev/null 2>&1; then
            test_result "NAVΛ Studio IDE" "PASS" "http://localhost:5175 accessible"
        else
            test_result "NAVΛ Studio IDE" "WARN" "IDE may not be fully ready"
        fi
    fi
else
    echo -e "${YELLOW}⚠${NC} No services running - skipping API tests"
    echo "  Start services with: ./start-robotis-system.sh"
fi

echo ""
echo "================================================"
echo "📊 Verification Summary"
echo "================================================"
echo ""
echo -e "${GREEN}Passed:${NC}   $PASSED"
echo -e "${YELLOW}Warnings:${NC} $WARNINGS"
echo -e "${RED}Failed:${NC}   $FAILED"
echo ""

if [ $FAILED -eq 0 ]; then
    if [ $WARNINGS -eq 0 ]; then
        echo -e "${GREEN}✅ Integration is FULLY VERIFIED!${NC}"
        echo ""
        if [ $SERVICES_RUNNING -eq 3 ]; then
            echo "All services are running. Access:"
            echo "  🌐 NAVΛ Studio IDE: http://localhost:5175/workspace.html"
            echo "  🔷 Univarm Console:  Click 'Univarm' icon in the dock"
        else
            echo "Integration files are ready. Start services with:"
            echo "  ./start-robotis-system.sh"
        fi
    else
        echo -e "${YELLOW}⚠ Integration is VERIFIED with warnings${NC}"
        echo ""
        echo "Review warnings above and fix if necessary."
        echo "System should still function properly."
    fi
else
    echo -e "${RED}❌ Integration verification FAILED${NC}"
    echo ""
    echo "Please fix the failed tests above before proceeding."
    exit 1
fi

echo ""

