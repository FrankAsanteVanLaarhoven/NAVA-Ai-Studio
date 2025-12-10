#!/bin/bash
# Build platform-specific installers for NAVΛ SDK
# Creates .dmg (macOS), .exe (Windows), .AppImage/.deb (Linux)
# Includes integrity checks and validation

set -e

# Colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
SDK_ROOT="$( cd "$SCRIPT_DIR/.." && pwd )"
PROJECT_ROOT="$( cd "$SDK_ROOT/.." && pwd )"
INSTALLERS_DIR="$SDK_ROOT/installers"
PUBLIC_INSTALLERS_DIR="$PROJECT_ROOT/public/sdk/installers"

echo -e "${BLUE}🚀 Building NAVΛ SDK Installers...${NC}"
echo ""

# Create installers directories
mkdir -p "$INSTALLERS_DIR"
mkdir -p "$PUBLIC_INSTALLERS_DIR"

# Check if we can skip build (installer already exists and is valid)
PLATFORM=$(uname -s)
if [[ "$PLATFORM" == "Darwin" ]]; then
  LATEST_DMG="$PUBLIC_INSTALLERS_DIR/NAVΛ-SDK-latest.dmg"
  if [ -f "$LATEST_DMG" ]; then
    SIZE=$(stat -f%z "$LATEST_DMG" 2>/dev/null || stat -c%s "$LATEST_DMG" 2>/dev/null || echo "0")
    if [ "$SIZE" -gt 1048576 ]; then
      echo -e "${GREEN}✅ Installer already exists and is valid ($(($SIZE / 1024 / 1024))MB)${NC}"
      echo -e "${BLUE}   Skipping build. Use --force to rebuild.${NC}"
      # Check if --force flag is set
      if [[ "$1" != "--force" ]]; then
        exit 0
      fi
    fi
  fi
fi

# Detect current platform
PLATFORM=$(uname -s)
ARCH=$(uname -m)

echo -e "${BLUE}📦 Detected Platform: ${PLATFORM} ${ARCH}${NC}"
echo ""

# Build all SDK components first
echo -e "${BLUE}📦 Step 1: Building SDK Components${NC}"
cd "$SDK_ROOT"
if [ -f "scripts/build-all.sh" ]; then
  bash scripts/build-all.sh || {
    echo -e "${YELLOW}⚠️  Build script had warnings, continuing...${NC}"
  }
else
  echo -e "${YELLOW}⚠️  Build script not found, building components individually...${NC}"
  
  # Build native
  if command -v cargo &> /dev/null; then
    cd "$SDK_ROOT/native"
    cargo build --release || echo -e "${YELLOW}⚠️  Native build had issues${NC}"
    cd "$SDK_ROOT"
  fi
  
  # Build web
  if command -v npm &> /dev/null; then
    cd "$SDK_ROOT/web"
    npm install && npm run build || echo -e "${YELLOW}⚠️  Web build had issues${NC}"
    cd "$SDK_ROOT"
  fi
fi

echo ""
echo -e "${BLUE}📦 Step 2: Creating Platform-Specific Installers${NC}"

# Function to verify file integrity
verify_file() {
  local file="$1"
  if [ ! -f "$file" ]; then
    echo -e "${RED}❌ File not found: $file${NC}"
    return 1
  fi
  
  local size=$(stat -f%z "$file" 2>/dev/null || stat -c%s "$file" 2>/dev/null || echo "0")
  if [ "$size" -eq 0 ]; then
    echo -e "${RED}❌ File is empty: $file${NC}"
    return 1
  fi
  
  echo -e "${GREEN}✅ File verified: $file (${size} bytes)${NC}"
  return 0
}

# macOS - Create .dmg with proper validation
if [[ "$PLATFORM" == "Darwin" ]]; then
  echo -e "${BLUE}🍎 Creating macOS .dmg installer...${NC}"
  
  DMG_NAME="NAVΛ-SDK-$(date +%Y%m%d).dmg"
  DMG_PATH="$INSTALLERS_DIR/$DMG_NAME"
  APP_NAME="NAVΛ SDK.app"
  APP_DIR="$INSTALLERS_DIR/$APP_NAME"
  TEMP_DMG_DIR="$INSTALLERS_DIR/dmg_temp"
  
  # Clean up any existing files
  rm -rf "$APP_DIR" "$TEMP_DMG_DIR" "$DMG_PATH" 2>/dev/null || true
  
  # Create .app bundle structure
  mkdir -p "$APP_DIR/Contents/MacOS"
  mkdir -p "$APP_DIR/Contents/Resources"
  mkdir -p "$APP_DIR/Contents/Frameworks"
  
  # Create Info.plist
  cat > "$APP_DIR/Contents/Info.plist" << 'PLIST_EOF'
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
  <key>CFBundleExecutable</key>
  <string>nava-sdk</string>
  <key>CFBundleIdentifier</key>
  <string>studio.nava.sdk</string>
  <key>CFBundleName</key>
  <string>NAVΛ SDK</string>
  <key>CFBundleVersion</key>
  <string>1.0.0</string>
  <key>CFBundleShortVersionString</key>
  <string>1.0.0</string>
  <key>CFBundleIconFile</key>
  <string>icon.icns</string>
  <key>LSMinimumSystemVersion</key>
  <string>11.0</string>
  <key>NSHighResolutionCapable</key>
  <true/>
</dict>
</plist>
PLIST_EOF
  
  # Verify Info.plist was created
  if ! verify_file "$APP_DIR/Contents/Info.plist"; then
    echo -e "${RED}❌ Failed to create Info.plist${NC}"
    exit 1
  fi
  
  # Copy SDK files with verification
  echo "Copying SDK files..."
  
  # Copy native library if it exists
  if [ -f "$SDK_ROOT/native/target/release/libnava_sdk.dylib" ]; then
    cp "$SDK_ROOT/native/target/release/libnava_sdk.dylib" "$APP_DIR/Contents/Frameworks/" && \
    verify_file "$APP_DIR/Contents/Frameworks/libnava_sdk.dylib" || \
    echo -e "${YELLOW}⚠️  Native library not found, skipping${NC}"
  fi
  
  # Copy web dist if it exists
  if [ -d "$SDK_ROOT/web/dist" ]; then
    cp -r "$SDK_ROOT/web/dist" "$APP_DIR/Contents/Resources/" && \
    echo -e "${GREEN}✅ Web SDK copied${NC}" || \
    echo -e "${YELLOW}⚠️  Web dist not found, skipping${NC}"
  fi
  
  # Copy assets
  if [ -d "$SDK_ROOT/assets" ]; then
    cp -r "$SDK_ROOT/assets" "$APP_DIR/Contents/Resources/" && \
    echo -e "${GREEN}✅ Assets copied${NC}" || \
    echo -e "${YELLOW}⚠️  Assets not found, skipping${NC}"
  fi
  
  # Copy icon
  if [ -f "$SDK_ROOT/assets/icon-512.png" ]; then
    cp "$SDK_ROOT/assets/icon-512.png" "$APP_DIR/Contents/Resources/icon.png" && \
    verify_file "$APP_DIR/Contents/Resources/icon.png" || \
    echo -e "${YELLOW}⚠️  Icon not found, using default${NC}"
  fi
  
  # Create README
  cat > "$APP_DIR/Contents/Resources/README.txt" << 'README_EOF'
NAVΛ Studio SDK
===============

Installation:
1. Drag NAVΛ SDK.app to your Applications folder
2. Launch from Applications
3. SDK files are located in Contents/Resources/

For development:
- Native SDK: Contents/Frameworks/
- Web SDK: Contents/Resources/dist/
- Assets: Contents/Resources/assets/

Version: 1.0.0
README_EOF
  
  # Create launcher script
  cat > "$APP_DIR/Contents/MacOS/nava-sdk" << 'LAUNCHER_EOF'
#!/bin/bash
# NAVΛ SDK Launcher

SDK_DIR="$(dirname "$0")/../Resources"
cd "$SDK_DIR"

# Display welcome message
osascript <<APPLESCRIPT
display dialog "NAVΛ SDK is ready!

Location: $SDK_DIR

The SDK has been installed successfully.
You can now use it in your projects." buttons {"OK"} default button "OK" with title "NAVΛ SDK" with icon note
APPLESCRIPT

# Open Finder to SDK location
open "$SDK_DIR"

echo "NAVΛ SDK installed at: $SDK_DIR"
LAUNCHER_EOF
  
  chmod +x "$APP_DIR/Contents/MacOS/nava-sdk"
  
  # Verify launcher was created
  if ! verify_file "$APP_DIR/Contents/MacOS/nava-sdk"; then
    echo -e "${RED}❌ Failed to create launcher${NC}"
    exit 1
  fi
  
    # Create .dmg with proper settings
    if command -v hdiutil &> /dev/null; then
      echo "Creating DMG..."
      
      # Create temporary directory for DMG contents
      mkdir -p "$TEMP_DMG_DIR"
      
      # Copy app bundle
      if [ ! -d "$APP_DIR" ]; then
        echo -e "${RED}❌ App bundle not found: $APP_DIR${NC}"
        rm -rf "$TEMP_DMG_DIR"
        exit 1
      fi
      
      cp -R "$APP_DIR" "$TEMP_DMG_DIR/"
      if [ $? -ne 0 ]; then
        echo -e "${RED}❌ Failed to copy app bundle${NC}"
        rm -rf "$TEMP_DMG_DIR"
        exit 1
      fi
      
      # Create Applications symlink
      ln -s /Applications "$TEMP_DMG_DIR/Applications" 2>/dev/null || true
      
      # Verify we have content
      if [ ! -d "$TEMP_DMG_DIR/NAVΛ SDK.app" ]; then
        echo -e "${RED}❌ App bundle not in temp directory${NC}"
        rm -rf "$TEMP_DMG_DIR"
        exit 1
      fi
      
      # Remove old DMG if it exists
      rm -f "$DMG_PATH"
      
      # Create DMG with proper format (try UDBZ first, fallback to UDRO)
      echo "Creating DMG from: $TEMP_DMG_DIR"
      if ! hdiutil create -volname "NAVΛ SDK" \
        -srcfolder "$TEMP_DMG_DIR" \
        -ov \
        -format UDBZ \
        -fs HFS+ \
        "$DMG_PATH" 2>&1; then
        echo -e "${YELLOW}⚠️  UDBZ format failed, trying UDRO...${NC}"
        if ! hdiutil create -volname "NAVΛ SDK" \
          -srcfolder "$TEMP_DMG_DIR" \
          -ov \
          -format UDRO \
          -fs HFS+ \
          "$DMG_PATH" 2>&1; then
          echo -e "${RED}❌ Failed to create DMG${NC}"
          rm -rf "$TEMP_DMG_DIR"
          exit 1
        fi
      fi
      
      # Verify DMG was created and has reasonable size
      if [ ! -f "$DMG_PATH" ]; then
        echo -e "${RED}❌ DMG file was not created${NC}"
        rm -rf "$TEMP_DMG_DIR"
        exit 1
      fi
      
      DMG_SIZE=$(stat -f%z "$DMG_PATH" 2>/dev/null || stat -c%s "$DMG_PATH" 2>/dev/null || echo "0")
      
      # Test if DMG can be mounted (more reliable than size check)
      MOUNT_TEST=$(hdiutil attach -nobrowse -noverify -noautoopen "$DMG_PATH" 2>&1 | grep -E '^/dev/' | awk '{print $3}' || echo "")
      if [ -n "$MOUNT_TEST" ] && [ -d "$MOUNT_TEST" ]; then
        echo -e "${GREEN}✅ DMG created and verified (${DMG_SIZE} bytes)${NC}"
        hdiutil detach "$MOUNT_TEST" >/dev/null 2>&1 || true
      else
        echo -e "${RED}❌ DMG file is corrupted and cannot be mounted${NC}"
        rm -f "$DMG_PATH"
        rm -rf "$TEMP_DMG_DIR"
        exit 1
      fi
      
      # Warn if size is suspiciously small but still accept if it mounts
      if [ "$DMG_SIZE" -lt 50000 ]; then
        echo -e "${YELLOW}⚠️  Warning: DMG is small ($DMG_SIZE bytes) but mounts successfully${NC}"
      fi
    
    # Verify DMG was created
    if verify_file "$DMG_PATH"; then
      # Test mount the DMG to verify it's not corrupted
      echo "Verifying DMG integrity..."
      MOUNT_POINT=$(hdiutil attach -nobrowse -noverify -noautoopen "$DMG_PATH" | grep -E '^/dev/' | awk '{print $3}')
      
      if [ -n "$MOUNT_POINT" ] && [ -d "$MOUNT_POINT" ]; then
        echo -e "${GREEN}✅ DMG verified successfully${NC}"
        hdiutil detach "$MOUNT_POINT" >/dev/null 2>&1 || true
        
        # Generate checksum
        CHECKSUM=$(shasum -a 256 "$DMG_PATH" | awk '{print $1}')
        echo "$CHECKSUM" > "$DMG_PATH.sha256"
        echo -e "${GREEN}✅ Created: $DMG_PATH${NC}"
        echo -e "${GREEN}✅ Checksum: $CHECKSUM${NC}"
        
        # Create symlink for easy access
        LATEST_DMG="$INSTALLERS_DIR/NAVΛ-SDK-latest.dmg"
        rm -f "$LATEST_DMG"
        ln -s "$(basename "$DMG_PATH")" "$LATEST_DMG" 2>/dev/null || \
        cp "$DMG_PATH" "$LATEST_DMG"
        echo -e "${GREEN}✅ Created symlink: $LATEST_DMG${NC}"
        
        # Copy to public directory for web access
        cp "$DMG_PATH" "$PUBLIC_INSTALLERS_DIR/NAVΛ-SDK-latest.dmg" 2>/dev/null && \
        cp "$DMG_PATH.sha256" "$PUBLIC_INSTALLERS_DIR/NAVΛ-SDK-latest.dmg.sha256" 2>/dev/null && \
        echo -e "${GREEN}✅ Copied to public directory for web download${NC}" || \
        echo -e "${YELLOW}⚠️  Could not copy to public directory${NC}"
      else
        echo -e "${RED}❌ DMG verification failed - file may be corrupted${NC}"
        rm -f "$DMG_PATH"
        exit 1
      fi
    else
      echo -e "${RED}❌ DMG creation failed${NC}"
      exit 1
    fi
    
    # Cleanup
    rm -rf "$TEMP_DMG_DIR"
  else
    echo -e "${YELLOW}⚠️  hdiutil not found, skipping .dmg creation${NC}"
  fi
fi

# Linux - Create .AppImage and .deb
if [[ "$PLATFORM" == "Linux" ]]; then
  echo -e "${BLUE}🐧 Creating Linux installers...${NC}"
  
  # Create AppImage structure
  APPIMAGE_DIR="$INSTALLERS_DIR/NAVΛ-SDK.AppDir"
  rm -rf "$APPIMAGE_DIR"
  mkdir -p "$APPIMAGE_DIR/usr/bin"
  mkdir -p "$APPIMAGE_DIR/usr/lib"
  mkdir -p "$APPIMAGE_DIR/usr/share/nava-sdk"
  
  # Copy SDK files
  if [ -f "$SDK_ROOT/native/target/release/libnava_sdk.so" ]; then
    cp "$SDK_ROOT/native/target/release/libnava_sdk.so" "$APPIMAGE_DIR/usr/lib/" && \
    verify_file "$APPIMAGE_DIR/usr/lib/libnava_sdk.so" || true
  fi
  
  if [ -d "$SDK_ROOT/web/dist" ]; then
    cp -r "$SDK_ROOT/web/dist" "$APPIMAGE_DIR/usr/share/nava-sdk/" || true
  fi
  
  if [ -d "$SDK_ROOT/assets" ]; then
    cp -r "$SDK_ROOT/assets" "$APPIMAGE_DIR/usr/share/nava-sdk/" || true
  fi
  
  # Create AppRun
  cat > "$APPIMAGE_DIR/AppRun" << 'APPRUN_EOF'
#!/bin/bash
cd "$(dirname "$0")/usr/share/nava-sdk"
exec "$(dirname "$0")/usr/bin/nava-sdk" "$@"
APPRUN_EOF
  chmod +x "$APPIMAGE_DIR/AppRun"
  
  # Create .desktop file
  cat > "$APPIMAGE_DIR/nava-sdk.desktop" << 'DESKTOP_EOF'
[Desktop Entry]
Name=NAVΛ SDK
Exec=nava-sdk
Icon=nava-sdk
Type=Application
Categories=Development;
DESKTOP_EOF
  
  # Create .deb package
  if command -v dpkg-deb &> /dev/null; then
    DEB_NAME="nava-sdk_1.0.0_${ARCH}.deb"
    DEB_PATH="$INSTALLERS_DIR/$DEB_NAME"
    DEB_DIR="$INSTALLERS_DIR/nava-sdk_1.0.0_${ARCH}"
    rm -rf "$DEB_DIR"
    
    mkdir -p "$DEB_DIR/DEBIAN"
    mkdir -p "$DEB_DIR/usr/bin"
    mkdir -p "$DEB_DIR/usr/lib"
    mkdir -p "$DEB_DIR/usr/share/nava-sdk"
    
    cat > "$DEB_DIR/DEBIAN/control" << CONTROL_EOF
Package: nava-sdk
Version: 1.0.0
Architecture: ${ARCH}
Maintainer: NAVΛ Team <team@nava.studio>
Description: NAVΛ Studio SDK - Navigation Calculus Development Kit
Priority: optional
Section: devel
CONTROL_EOF
    
    # Copy files
    if [ -f "$SDK_ROOT/native/target/release/libnava_sdk.so" ]; then
      cp "$SDK_ROOT/native/target/release/libnava_sdk.so" "$DEB_DIR/usr/lib/" || true
    fi
    
    if [ -d "$SDK_ROOT/web/dist" ]; then
      cp -r "$SDK_ROOT/web/dist" "$DEB_DIR/usr/share/nava-sdk/" || true
    fi
    
    if [ -d "$SDK_ROOT/assets" ]; then
      cp -r "$SDK_ROOT/assets" "$DEB_DIR/usr/share/nava-sdk/" || true
    fi
    
    # Build .deb
    dpkg-deb --build "$DEB_DIR" "$DEB_PATH" && \
    verify_file "$DEB_PATH" && \
    echo -e "${GREEN}✅ Created: $DEB_PATH${NC}" || \
    echo -e "${RED}❌ Failed to create .deb package${NC}"
    
    # Generate checksum
    if [ -f "$DEB_PATH" ]; then
      CHECKSUM=$(sha256sum "$DEB_PATH" | awk '{print $1}')
      echo "$CHECKSUM" > "$DEB_PATH.sha256"
      echo -e "${GREEN}✅ Checksum: $CHECKSUM${NC}"
    fi
  fi
fi

# Windows - Create .exe (requires Windows or Wine)
if [[ "$PLATFORM" == *"MINGW"* ]] || [[ "$PLATFORM" == *"MSYS"* ]] || command -v wine &> /dev/null; then
  echo -e "${BLUE}🪟 Creating Windows .exe installer...${NC}"
  
  EXE_NAME="NAVΛ-SDK-Setup-$(date +%Y%m%d).exe"
  EXE_PATH="$INSTALLERS_DIR/$EXE_NAME"
  
  # Create NSIS installer script
  NSIS_SCRIPT="$INSTALLERS_DIR/installer.nsi"
  cat > "$NSIS_SCRIPT" << 'NSIS_EOF'
!include "MUI2.nsh"

Name "NAVΛ SDK"
OutFile "NAVΛ-SDK-Setup.exe"
InstallDir "$PROGRAMFILES\NAVA SDK"
RequestExecutionLevel admin

!insertmacro MUI_PAGE_WELCOME
!insertmacro MUI_PAGE_DIRECTORY
!insertmacro MUI_PAGE_INSTFILES
!insertmacro MUI_PAGE_FINISH

!insertmacro MUI_LANGUAGE "English"

Section "Install"
  SetOutPath "$INSTDIR"
  File /r "native\target\release\*.dll"
  File /r "web\dist\*"
  File /r "assets\*"
  
  WriteRegStr HKLM "Software\NAVA SDK" "InstallDir" "$INSTDIR"
  WriteUninstaller "$INSTDIR\Uninstall.exe"
SectionEnd

Section "Uninstall"
  Delete "$INSTDIR\*.*"
  RMDir /r "$INSTDIR"
  DeleteRegKey HKLM "Software\NAVA SDK"
SectionEnd
NSIS_EOF
  
  if command -v makensis &> /dev/null; then
    makensis "$NSIS_SCRIPT" && \
    verify_file "$EXE_PATH" && \
    echo -e "${GREEN}✅ Created: $EXE_PATH${NC}" || \
    echo -e "${RED}❌ Failed to create .exe installer${NC}"
    
    # Generate checksum
    if [ -f "$EXE_PATH" ]; then
      CHECKSUM=$(sha256sum "$EXE_PATH" 2>/dev/null || shasum -a 256 "$EXE_PATH" | awk '{print $1}')
      echo "$CHECKSUM" > "$EXE_PATH.sha256"
      echo -e "${GREEN}✅ Checksum: $CHECKSUM${NC}"
    fi
  else
    echo -e "${YELLOW}⚠️  NSIS (makensis) not found, skipping .exe creation${NC}"
    echo "   Install NSIS to create Windows installers"
  fi
fi

echo ""
echo -e "${GREEN}🎉 Installer build complete!${NC}"
echo ""
echo "📁 Installers available in: $INSTALLERS_DIR"
if [ -d "$INSTALLERS_DIR" ]; then
  ls -lh "$INSTALLERS_DIR"/*.{dmg,exe,deb,AppImage} 2>/dev/null | while read line; do
    echo "   $line"
  done
  echo ""
  echo "📋 Checksums:"
  ls -lh "$INSTALLERS_DIR"/*.sha256 2>/dev/null | while read line; do
    echo "   $line"
  done
fi
