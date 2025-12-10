#!/bin/bash
# Ensure SDK installer is built and available for download
# This runs automatically during build to ensure installers are always ready

set -e

# Colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m'

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_ROOT="$( cd "$SCRIPT_DIR/.." && pwd )"
SDK_ROOT="$PROJECT_ROOT/sdk"
INSTALLERS_DIR="$SDK_ROOT/installers"
PUBLIC_INSTALLERS_DIR="$PROJECT_ROOT/public/sdk/installers"

# Create directories
mkdir -p "$INSTALLERS_DIR"
mkdir -p "$PUBLIC_INSTALLERS_DIR"

# Check if installer already exists and is valid
PLATFORM=$(uname -s)
if [[ "$PLATFORM" == "Darwin" ]]; then
  INSTALLER_FILE="$PUBLIC_INSTALLERS_DIR/NAVΛ-SDK-latest.dmg"
  IDE_INSTALLER_FILE="$PROJECT_ROOT/public/NAVΛ-Studio-IDE.dmg"
  
  # Check if installer exists and is valid (at least 1MB)
  if [ -f "$INSTALLER_FILE" ]; then
    SIZE=$(stat -f%z "$INSTALLER_FILE" 2>/dev/null || stat -c%s "$INSTALLER_FILE" 2>/dev/null || echo "0")
    if [ "$SIZE" -gt 1048576 ]; then
      echo -e "${GREEN}✅ Installer ready: $INSTALLER_FILE ($(($SIZE / 1024 / 1024))MB)${NC}"
      exit 0
    else
      echo -e "${YELLOW}⚠️  Installer exists but is too small, rebuilding...${NC}"
      rm -f "$INSTALLER_FILE"
    fi
  fi
  
  # Build installer
  echo -e "${BLUE}🔨 Building SDK installer for download...${NC}"
  cd "$SDK_ROOT"
  
  # Run build script
  if [ -f "scripts/build-installers.sh" ]; then
    bash scripts/build-installers.sh || {
      echo -e "${YELLOW}⚠️  Installer build had warnings, but continuing...${NC}"
    }
  else
    echo -e "${YELLOW}⚠️  Build script not found${NC}"
    exit 1
  fi
  
  # Verify installer was created
  SOURCE_FILE="$INSTALLERS_DIR/NAVΛ-SDK-latest.dmg"
  if [ -f "$SOURCE_FILE" ]; then
    SIZE=$(stat -f%z "$SOURCE_FILE" 2>/dev/null || stat -c%s "$SOURCE_FILE" 2>/dev/null || echo "0")
    if [ "$SIZE" -gt 1048576 ]; then
      echo -e "${GREEN}✅ Installer built: $SOURCE_FILE ($(($SIZE / 1024 / 1024))MB)${NC}"
      
      # Copy to public directory for web access
      cp "$SOURCE_FILE" "$INSTALLER_FILE" 2>/dev/null && \
      echo -e "${GREEN}✅ Installer ready for download at: $INSTALLER_FILE${NC}" || \
      echo -e "${YELLOW}⚠️  Could not copy to public directory${NC}"
      
      # Also create IDE installer link/copy for direct download
      cp "$SOURCE_FILE" "$IDE_INSTALLER_FILE" 2>/dev/null && \
      echo -e "${GREEN}✅ IDE installer ready at: $IDE_INSTALLER_FILE${NC}" || \
      echo -e "${YELLOW}⚠️  Could not create IDE installer link${NC}"
    else
      echo -e "${YELLOW}⚠️  Installer file is too small, may be corrupted${NC}"
      exit 1
    fi
  else
    echo -e "${YELLOW}⚠️  Installer was not created${NC}"
    exit 1
  fi
else
  echo -e "${YELLOW}⚠️  Installer build only supported on macOS currently${NC}"
  exit 0
fi

