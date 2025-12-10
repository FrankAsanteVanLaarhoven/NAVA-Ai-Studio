#!/bin/bash
# Setup NAVA Model from Colab Download
# This script helps set up the model after downloading from Colab

set -e

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}📥 NAVA Model Setup from Colab${NC}"
echo ""

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
MODELS_DIR="$PROJECT_ROOT/models"

echo -e "${YELLOW}Step 1: Checking for downloaded model...${NC}"

# Check if model is already in models directory
if [ -d "$MODELS_DIR/nava-llama-3.1-8b-instruct-merged" ]; then
    echo -e "${GREEN}✅ Found model at: $MODELS_DIR/nava-llama-3.1-8b-instruct-merged${NC}"
    MODEL_PATH="$MODELS_DIR/nava-llama-3.1-8b-instruct-merged"
elif [ -d "$MODELS_DIR/nava-instruct-7b-merged" ]; then
    echo -e "${GREEN}✅ Found model at: $MODELS_DIR/nava-instruct-7b-merged${NC}"
    MODEL_PATH="$MODELS_DIR/nava-instruct-7b-merged"
else
    echo -e "${YELLOW}⚠️  Model not found in models directory${NC}"
    echo ""
    echo "Please download the model from Colab first:"
    echo "  1. In Colab, go to /content/nava-llama-3.1-8b-instruct-merged/"
    echo "  2. Right-click → Download"
    echo "  3. Extract to: $MODELS_DIR/"
    echo ""
    echo "Or if you have the model elsewhere, specify the path:"
    read -p "Enter model path (or press Enter to exit): " MODEL_PATH
    
    if [ -z "$MODEL_PATH" ]; then
        echo "Exiting..."
        exit 1
    fi
fi

# Verify model directory
if [ ! -d "$MODEL_PATH" ]; then
    echo -e "${RED}❌ Error: Model directory not found: $MODEL_PATH${NC}"
    exit 1
fi

if [ ! -f "$MODEL_PATH/config.json" ]; then
    echo -e "${RED}❌ Error: Invalid model directory (missing config.json)${NC}"
    exit 1
fi

echo -e "${GREEN}✅ Model verified!${NC}"
echo ""

echo -e "${YELLOW}Step 2: Creating backup copy on Desktop...${NC}"

# Create backup on Desktop
DESKTOP_BACKUP="$HOME/Desktop/nava-llama-3.1-8b-instruct-merged"
if [ ! -d "$DESKTOP_BACKUP" ]; then
    echo "Copying model to Desktop backup..."
    cp -r "$MODEL_PATH" "$DESKTOP_BACKUP"
    echo -e "${GREEN}✅ Backup created at: $DESKTOP_BACKUP${NC}"
else
    echo -e "${YELLOW}⚠️  Backup already exists at: $DESKTOP_BACKUP${NC}"
    read -p "Overwrite? (y/N): " OVERWRITE
    if [ "$OVERWRITE" = "y" ] || [ "$OVERWRITE" = "Y" ]; then
        rm -rf "$DESKTOP_BACKUP"
        cp -r "$MODEL_PATH" "$DESKTOP_BACKUP"
        echo -e "${GREEN}✅ Backup updated${NC}"
    fi
fi

echo ""
echo -e "${YELLOW}Step 3: Setting up model path...${NC}"

# Create .env file with model path
ENV_FILE="$PROJECT_ROOT/.env"
if [ ! -f "$ENV_FILE" ]; then
    touch "$ENV_FILE"
fi

# Add or update NAVA_MODEL_PATH
if grep -q "NAVA_MODEL_PATH" "$ENV_FILE"; then
    # Update existing
    sed -i.bak "s|NAVA_MODEL_PATH=.*|NAVA_MODEL_PATH=$MODEL_PATH|" "$ENV_FILE"
    echo -e "${GREEN}✅ Updated NAVA_MODEL_PATH in .env${NC}"
else
    # Add new
    echo "NAVA_MODEL_PATH=$MODEL_PATH" >> "$ENV_FILE"
    echo -e "${GREEN}✅ Added NAVA_MODEL_PATH to .env${NC}"
fi

echo ""
echo -e "${GREEN}✅ Setup Complete!${NC}"
echo ""
echo "Model location: $MODEL_PATH"
echo "Backup location: $DESKTOP_BACKUP"
echo ""
echo "To start the server, run:"
echo "  cd \"$PROJECT_ROOT\""
echo "  ./scripts/start_nava_model.sh"
echo ""
echo "Or manually:"
echo "  export NAVA_MODEL_PATH=\"$MODEL_PATH\""
echo "  python3 scripts/serve_nava_model.py --backend huggingface --port 8080"
