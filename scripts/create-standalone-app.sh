#!/bin/bash
# Create standalone NAVΛ Studio app for distribution

set -e

echo "📦 Creating standalone NAVΛ Studio app..."

PROJECT_DIR="/Users/frankvanlaarhoven/Desktop/NAVΛ STUDIO IDE"
APP_NAME="NAVΛ Studio.app"
DIST_DIR="$PROJECT_DIR/dist-app"
APP_BUNDLE="$DIST_DIR/$APP_NAME"

# Clean and create distribution directory
rm -rf "$DIST_DIR"
mkdir -p "$DIST_DIR"

# Create app bundle structure
mkdir -p "$APP_BUNDLE/Contents/MacOS"
mkdir -p "$APP_BUNDLE/Contents/Resources"
mkdir -p "$APP_BUNDLE/Contents/Resources/app"

# Build the production app
echo "🔨 Building production bundle..."
cd "$PROJECT_DIR"
npm run build

# Copy built files
echo "📋 Copying application files..."
cp -r dist/* "$APP_BUNDLE/Contents/Resources/app/"
cp -r node_modules "$APP_BUNDLE/Contents/Resources/"
cp package.json "$APP_BUNDLE/Contents/Resources/"

# Create launcher script
cat > "$APP_BUNDLE/Contents/MacOS/navlambda-studio" << 'LAUNCHER_EOF'
#!/bin/bash

# Get paths
BUNDLE_PATH="$(cd "$(dirname "$0")/../.." && pwd)"
RESOURCES_PATH="$BUNDLE_PATH/Contents/Resources"
APP_PATH="$RESOURCES_PATH/app"

# Change to resources directory
cd "$RESOURCES_PATH"

# Check if Node is installed
if ! command -v node &> /dev/null; then
    osascript -e 'display dialog "Node.js is required to run NAVΛ Studio.\n\nPlease install Node.js from nodejs.org" buttons {"OK"} default button 1 with icon stop'
    open "https://nodejs.org"
    exit 1
fi

# Start a simple HTTP server on a random available port
PORT=3000
while lsof -Pi :$PORT -sTCP:LISTEN -t >/dev/null ; do
    PORT=$((PORT + 1))
done

echo "Starting NAVΛ Studio on port $PORT..."

# Start server in background
cd "$APP_PATH"
python3 -m http.server $PORT > /tmp/navlambda-studio.log 2>&1 &
SERVER_PID=$!

# Wait for server to start
sleep 2

# Open in Chrome app mode or default browser
if [ -d "/Applications/Google Chrome.app" ]; then
    open -n -a "Google Chrome" --args --app="http://localhost:$PORT" --new-window
elif [ -d "/Applications/Arc.app" ]; then
    open -a "Arc" "http://localhost:$PORT"
else
    open "http://localhost:$PORT"
fi

# Show notification
osascript -e 'display notification "NAVΛ Studio is running at http://localhost:'$PORT'" with title "NAVΛ Studio"'

# Keep server running
wait $SERVER_PID
LAUNCHER_EOF

chmod +x "$APP_BUNDLE/Contents/MacOS/navlambda-studio"

# Create Info.plist
cat > "$APP_BUNDLE/Contents/Info.plist" << 'PLIST_EOF'
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
    <key>CFBundleExecutable</key>
    <string>navlambda-studio</string>
    <key>CFBundleIconFile</key>
    <string>icon</string>
    <key>CFBundleIdentifier</key>
    <string>studio.navlambda.ide</string>
    <key>CFBundleName</key>
    <string>NAVΛ Studio</string>
    <key>CFBundleDisplayName</key>
    <string>NAVΛ Studio</string>
    <key>CFBundlePackageType</key>
    <string>APPL</string>
    <key>CFBundleShortVersionString</key>
    <string>1.0.0</string>
    <key>CFBundleVersion</key>
    <string>1</string>
    <key>LSMinimumSystemVersion</key>
    <string>10.15</string>
    <key>NSHighResolutionCapable</key>
    <true/>
    <key>LSApplicationCategoryType</key>
    <string>public.app-category.developer-tools</string>
</dict>
</plist>
PLIST_EOF

# Copy icon
if [ -f "$PROJECT_DIR/src-tauri/icons/icon.png" ]; then
    cp "$PROJECT_DIR/src-tauri/icons/icon.png" "$APP_BUNDLE/Contents/Resources/icon.png"
fi

# Create DMG for distribution
echo "💿 Creating DMG installer..."
DMG_NAME="NAVΛ-Studio-1.0.0-macOS.dmg"
DMG_PATH="$DIST_DIR/$DMG_NAME"

# Remove old DMG if exists
rm -f "$DMG_PATH"

# Create DMG
hdiutil create -volname "NAVΛ Studio" -srcfolder "$APP_BUNDLE" -ov -format UDZO "$DMG_PATH"

# Create ZIP for easy download
echo "📦 Creating ZIP archive..."
cd "$DIST_DIR"
zip -r "NAVΛ-Studio-1.0.0-macOS.zip" "$APP_NAME"

echo "✅ Done!"
echo ""
echo "📦 Distribution files created:"
echo "   App Bundle: $APP_BUNDLE"
echo "   DMG: $DMG_PATH"
echo "   ZIP: $DIST_DIR/NAVΛ-Studio-1.0.0-macOS.zip"
echo ""
echo "🚀 To install: Open the DMG and drag NAVΛ Studio to Applications"

