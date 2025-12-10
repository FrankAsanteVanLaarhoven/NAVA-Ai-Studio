#!/bin/bash
# Create a complete, ready-to-download installer package

set -e

PROJECT_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"
INSTALLER_DIR="$PROJECT_ROOT/public"
INSTALLER_NAME="NAVΛ-Studio-IDE.dmg"

echo "📦 Creating complete installer package..."

# Create a simple app bundle structure
APP_DIR="$INSTALLER_DIR/NAVΛ Studio IDE.app"
rm -rf "$APP_DIR"
mkdir -p "$APP_DIR/Contents/MacOS"
mkdir -p "$APP_DIR/Contents/Resources"

# Create Info.plist
cat > "$APP_DIR/Contents/Info.plist" << PLIST_EOF
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
  <key>CFBundleExecutable</key>
  <string>nava-studio-ide</string>
  <key>CFBundleIdentifier</key>
  <string>studio.nava.ide</string>
  <key>CFBundleName</key>
  <string>NAVΛ Studio IDE</string>
  <key>CFBundleVersion</key>
  <string>1.0.0</string>
  <key>CFBundleShortVersionString</key>
  <string>1.0.0</string>
  <key>LSMinimumSystemVersion</key>
  <string>11.0</string>
  <key>NSHighResolutionCapable</key>
  <true/>
</dict>
</plist>
PLIST_EOF

# Create launcher script
cat > "$APP_DIR/Contents/MacOS/nava-studio-ide" << 'LAUNCHER_EOF'
#!/bin/bash
# Launch NAVΛ Studio IDE
open "http://localhost:5173/workspace.html" || open "https://navlambda.studio"
LAUNCHER_EOF

chmod +x "$APP_DIR/Contents/MacOS/nava-studio-ide"

# Copy icon if available
if [ -f "$PROJECT_ROOT/public/favicon.svg" ]; then
  cp "$PROJECT_ROOT/public/favicon.svg" "$APP_DIR/Contents/Resources/icon.svg" 2>/dev/null || true
fi

# Create DMG
DMG_PATH="$INSTALLER_DIR/$INSTALLER_NAME"
TEMP_DMG_DIR="$INSTALLER_DIR/dmg_temp"

rm -rf "$TEMP_DMG_DIR" "$DMG_PATH"
mkdir -p "$TEMP_DMG_DIR"

cp -R "$APP_DIR" "$TEMP_DMG_DIR/"
ln -s /Applications "$TEMP_DMG_DIR/Applications"

# Create DMG using hdiutil
hdiutil create -volname "NAVΛ Studio IDE" \
  -srcfolder "$TEMP_DMG_DIR" \
  -ov \
  -format UDBZ \
  -fs HFS+ \
  "$DMG_PATH" 2>&1

# Verify DMG
if [ -f "$DMG_PATH" ]; then
  SIZE=$(stat -f%z "$DMG_PATH" 2>/dev/null || stat -c%s "$DMG_PATH" 2>/dev/null || echo "0")
  if [ "$SIZE" -gt 100000 ]; then
    echo "✅ Complete installer created: $DMG_PATH ($(($SIZE / 1024 / 1024))MB)"
    rm -rf "$TEMP_DMG_DIR"
    exit 0
  else
    echo "❌ DMG too small: $SIZE bytes"
    rm -f "$DMG_PATH"
    exit 1
  fi
else
  echo "❌ Failed to create DMG"
  exit 1
fi
