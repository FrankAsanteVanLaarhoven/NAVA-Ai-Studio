#!/bin/bash

# Simple build verification script for the NAVΛ Simulation Platform

echo "Verifying NAVΛ Simulation Platform structure..."

# Check if the required directories exist
if [ ! -d "simulation-platform" ]; then
    echo "Error: simulation-platform directory not found"
    exit 1
fi

if [ ! -d "simulation-platform/src" ]; then
    echo "Error: simulation-platform/src directory not found"
    exit 1
fi

# Check if the main files exist
files_to_check=(
    "simulation-platform/Cargo.toml"
    "simulation-platform/src/main.rs"
    "simulation-platform/src/core/mod.rs"
    "simulation-platform/src/physics/mod.rs"
    "simulation-platform/src/rendering/mod.rs"
    "simulation-platform/src/robotics/mod.rs"
    "simulation-platform/src/ros2/mod.rs"
    "simulation-platform/config.json"
    "simulation-platform/README.md"
)

for file in "${files_to_check[@]}"; do
    if [ ! -f "$file" ]; then
        echo "Error: $file not found"
        exit 1
    fi
done

echo "All required files found. Structure verification successful!"

# Display the project structure
echo ""
echo "Project Structure:"
echo "├── Cargo.toml"
echo "├── config.json"
echo "├── README.md"
echo "└── src/"
echo "    ├── main.rs"
echo "    ├── core/"
echo "    │   └── mod.rs"
echo "    ├── physics/"
echo "    │   └── mod.rs"
echo "    ├── rendering/"
echo "    │   └── mod.rs"
echo "    ├── robotics/"
echo "    │   └── mod.rs"
echo "    └── ros2/"
echo "        └── mod.rs"

echo ""
echo "The NAVΛ Simulation Platform is ready for development!"
echo "Run 'cargo build' in the simulation-platform directory when your Rust environment is properly configured."