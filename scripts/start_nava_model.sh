#!/bin/bash
# -*- coding: utf-8 -*-
# Start NAVA-Instruct Model Server
# This script starts the fine-tuned NAVA-Instruct model server

set -e

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${GREEN}🚀 Starting NAVA-Instruct Model Server${NC}"
echo ""

# Check if Python is available
if ! command -v python3 &> /dev/null; then
    echo -e "${RED}❌ Error: python3 not found${NC}"
    exit 1
fi

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

# Default values
MODEL_PATH="${NAVA_MODEL_PATH:-}"
PORT="${NAVA_MODEL_PORT:-8080}"
BACKEND="${NAVA_MODEL_BACKEND:-huggingface}"

# Check for model path argument
if [ -n "$1" ]; then
    MODEL_PATH="$1"
fi

# Check for port argument
if [ -n "$2" ]; then
    PORT="$2"
fi

# Check for backend argument
if [ -n "$3" ]; then
    BACKEND="$3"
fi

echo -e "${YELLOW}Configuration:${NC}"
echo "  Model Path: ${MODEL_PATH:-auto-detect}"
echo "  Port: $PORT"
echo "  Backend: $BACKEND"
echo ""

# Check if model path is set, otherwise auto-detect
if [ -z "$MODEL_PATH" ]; then
    echo -e "${YELLOW}🔍 Auto-detecting model path...${NC}"
    
    # Try common locations
    POSSIBLE_PATHS=(
        "$PROJECT_ROOT/models/nava-llama-3.1-8b-instruct-merged"
        "$PROJECT_ROOT/models/nava-instruct-7b-merged"
        "$(dirname "$PROJECT_ROOT")/LLM_Training_Notebook/models/nava-llama-3.1-8b-instruct-merged"
        "./models/nava-llama-3.1-8b-instruct-merged"
        "./models/nava-instruct-7b-merged"
    )
    
    for path in "${POSSIBLE_PATHS[@]}"; do
        if [ -d "$path" ] && [ -f "$path/config.json" ]; then
            MODEL_PATH="$path"
            echo -e "${GREEN}✅ Found model at: $MODEL_PATH${NC}"
            break
        fi
    done
    
    if [ -z "$MODEL_PATH" ]; then
        echo -e "${RED}❌ Error: NAVA-Instruct model not found${NC}"
        echo ""
        echo "Please specify model path:"
        echo "  ./start_nava_model.sh /path/to/model"
        echo ""
        echo "Or set environment variable:"
        echo "  export NAVA_MODEL_PATH=/path/to/model"
        echo ""
        echo "Or place model in one of:"
        for path in "${POSSIBLE_PATHS[@]}"; do
            echo "  - $path"
        done
        exit 1
    fi
fi

# Verify model exists
if [ ! -d "$MODEL_PATH" ]; then
    echo -e "${RED}❌ Error: Model directory not found: $MODEL_PATH${NC}"
    exit 1
fi

if [ ! -f "$MODEL_PATH/config.json" ]; then
    echo -e "${RED}❌ Error: Invalid model directory (missing config.json): $MODEL_PATH${NC}"
    exit 1
fi

# Check if port is available
if lsof -Pi :$PORT -sTCP:LISTEN -t >/dev/null 2>&1 ; then
    echo -e "${YELLOW}⚠️  Warning: Port $PORT is already in use${NC}"
    echo "   You may need to stop the existing server or use a different port"
    echo ""
fi

# Start the server
echo -e "${GREEN}🚀 Starting server...${NC}"
echo ""

cd "$SCRIPT_DIR"

if [ -n "$MODEL_PATH" ]; then
    python3 serve_nava_model.py \
        --model "$MODEL_PATH" \
        --port "$PORT" \
        --backend "$BACKEND"
else
    python3 serve_nava_model.py \
        --port "$PORT" \
        --backend "$BACKEND"
fi

