@echo off
REM NAVΛ SIM Platform Startup Script for Windows
REM Builds and runs the Rust-based simulation backend

echo.
echo 🚀 Starting NAVΛ Simulation Platform...
echo.

REM Check if Rust is installed
where cargo >nul 2>nul
if %ERRORLEVEL% NEQ 0 (
    echo ❌ Rust is not installed!
    echo 📦 Install Rust from: https://rustup.rs/
    pause
    exit /b 1
)

echo ✅ Rust detected
cargo --version
echo.

REM Navigate to simulation platform directory
cd simulation-platform

REM Build the project
echo 🔧 Building simulation platform...
cargo build --release

if %ERRORLEVEL% EQU 0 (
    echo.
    echo ✅ Build successful!
    echo.
    echo 🌐 Starting simulation API server on http://localhost:3030
    echo 🎮 Open NAVΛ Studio IDE and click the Simulation icon
    echo.
    echo ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    echo.
    
    REM Start the simulation server
    cargo run --release -- --api-only --verbose
) else (
    echo.
    echo ❌ Build failed! Check the errors above.
    pause
    exit /b 1
)

