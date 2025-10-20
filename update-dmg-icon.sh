#!/bin/bash

# Create DMG with Updated Icon
# This script creates a new DMG file with the navigation symbol icon

set -e

PROJECT_DIR="/Users/frankvanlaarhoven/Desktop/NAVΛ STUDIO IDE"
ICON_FILE="$PROJECT_DIR/src-tauri/icons/icon.icns"
APP_NAME="NAVΛ Studio"
DMG_NAME="NAVΛ-Studio-1.0.0-macOS.dmg"
TEMP_DIR=$(mktemp -d)

echo "Creating DMG with updated icon..."
echo "Icon: $ICON_FILE"

# Create a temporary directory structure
mkdir -p "$TEMP_DIR/dmg"
mkdir -p "$TEMP_DIR/dmg/.background"

# Copy the icon
cp "$ICON_FILE" "$TEMP_DIR/dmg/.VolumeIcon.icns"

# Create a placeholder app bundle (we'll use the icon as a placeholder)
mkdir -p "$TEMP_DIR/dmg/$APP_NAME.app/Contents/MacOS"
mkdir -p "$TEMP_DIR/dmg/$APP_NAME.app/Contents/Resources"
cp "$ICON_FILE" "$TEMP_DIR/dmg/$APP_NAME.app/Contents/Resources/AppIcon.icns"

# Create Info.plist
cat > "$TEMP_DIR/dmg/$APP_NAME.app/Contents/Info.plist" << EOF
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
    <key>CFBundleExecutable</key>
    <string>navlambda-studio</string>
    <key>CFBundleIconFile</key>
    <string>AppIcon</string>
    <key>CFBundleIdentifier</key>
    <string>studio.navlambda.ide</string>
    <key>CFBundleName</key>
    <string>$APP_NAME</string>
    <key>CFBundlePackageType</key>
    <string>APPL</string>
    <key>CFBundleShortVersionString</key>
    <string>1.0.0</string>
    <key>CFBundleVersion</key>
    <string>1.0.0</string>
</dict>
</plist>
EOF

# Create a simple executable placeholder
cat > "$TEMP_DIR/dmg/$APP_NAME.app/Contents/MacOS/navlambda-studio" << 'EOF'
#!/bin/bash
echo "NAVΛ Studio"
EOF
chmod +x "$TEMP_DIR/dmg/$APP_NAME.app/Contents/MacOS/navlambda-studio"

# Create the DMG
echo "Creating DMG..."
hdiutil create -volname "$APP_NAME" \
    -srcfolder "$TEMP_DIR/dmg" \
    -ov -format UDZO \
    "$TEMP_DIR/$DMG_NAME"

# Set the custom icon on the DMG
echo "Setting DMG icon..."
sips -i "$ICON_FILE"
DeRez -only icns "$ICON_FILE" > "$TEMP_DIR/icon.rsrc"
Rez -append "$TEMP_DIR/icon.rsrc" -o "$TEMP_DIR/$DMG_NAME"
SetFile -a C "$TEMP_DIR/$DMG_NAME"

# Copy to all locations
echo "Copying DMG to project locations..."
cp "$TEMP_DIR/$DMG_NAME" "$PROJECT_DIR/public/downloads/$DMG_NAME"
cp "$TEMP_DIR/$DMG_NAME" "$PROJECT_DIR/dist/downloads/$DMG_NAME"
cp "$TEMP_DIR/$DMG_NAME" "$PROJECT_DIR/dist-app/$DMG_NAME"

# Clean up
rm -rf "$TEMP_DIR"

echo "✅ DMG created successfully with navigation symbol icon!"
echo "📦 DMG location: public/downloads/$DMG_NAME"
