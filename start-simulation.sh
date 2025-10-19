#!/bin/bash

# NAVΛ SIM Platform Startup Script
# Builds and runs the Rust-based simulation backend

echo "🚀 Starting NAVΛ Simulation Platform..."
echo ""

# Check if Rust is installed
if ! command -v cargo &> /dev/null; then
    echo "❌ Rust is not installed!"
    echo "📦 Install Rust from: https://rustup.rs/"
    exit 1
fi

echo "✅ Rust detected: $(rustc --version)"
echo ""

# Navigate to simulation platform directory
cd simulation-platform || exit

# Check if this is first run (no target directory)
if [ ! -d "target" ]; then
    echo "📦 First run detected - building simulation platform..."
    echo "⏳ This may take a few minutes..."
    echo ""
    cargo build --release
else
    echo "🔧 Building simulation platform..."
    cargo build --release
fi

if [ $? -eq 0 ]; then
    echo ""
    echo "✅ Build successful!"
    echo ""
    echo "🌐 Starting simulation API server on http://localhost:3030"
    echo "🎮 Open NAVΛ Studio IDE and click the Simulation icon"
    echo ""
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo ""
    
    # Start the simulation server
    cargo run --release -- --api-only --verbose
else
    echo ""
    echo "❌ Build failed! Check the errors above."
    exit 1
fi

