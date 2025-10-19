#!/bin/bash
# NAVΛ Studio Desktop Launcher

echo "🚀 Starting NAVΛ Studio..."

# Start the dev server in the background
cd "/Users/frankvanlaarhoven/Desktop/NAVΛ STUDIO IDE"
npm run dev > /dev/null 2>&1 &
SERVER_PID=$!

# Wait for server to start
sleep 3

# Open in app mode (standalone window, no browser UI)
open -n -a "Google Chrome" --args --app="http://localhost:3000" --new-window

echo "✅ NAVΛ Studio is running!"
echo "Close this window to stop the server."

# Keep script running
wait $SERVER_PID

