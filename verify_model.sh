#!/bin/bash
# Verify NAVA Model Files
# Checks if downloaded model is complete and valid

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}🔍 Verifying NAVA Model${NC}"
echo ""

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
MODELS_DIR="$PROJECT_ROOT/models"

# Check for model
MODEL_PATH=""
if [ -d "$MODELS_DIR/nava-llama-3.1-8b-instruct-merged" ]; then
    MODEL_PATH="$MODELS_DIR/nava-llama-3.1-8b-instruct-merged"
elif [ -d "$MODELS_DIR/nava-instruct-7b-merged" ]; then
    MODEL_PATH="$MODELS_DIR/nava-instruct-7b-merged"
elif [ -n "$1" ]; then
    MODEL_PATH="$1"
else
    echo -e "${RED}❌ Model not found${NC}"
    echo ""
    echo "Expected locations:"
    echo "  - $MODELS_DIR/nava-llama-3.1-8b-instruct-merged"
    echo "  - $MODELS_DIR/nava-instruct-7b-merged"
    echo ""
    echo "Usage: $0 [model_path]"
    exit 1
fi

echo -e "${YELLOW}Checking: $MODEL_PATH${NC}"
echo ""

# Required files
REQUIRED_FILES=(
    "config.json"
    "generation_config.json"
)

# Optional but important files
IMPORTANT_FILES=(
    "tokenizer.json"
    "tokenizer_config.json"
    "model.safetensors"
    "pytorch_model.bin"
)

# Check required files
echo -e "${BLUE}Required Files:${NC}"
MISSING_REQUIRED=0
for file in "${REQUIRED_FILES[@]}"; do
    if [ -f "$MODEL_PATH/$file" ]; then
        SIZE=$(du -h "$MODEL_PATH/$file" | cut -f1)
        echo -e "  ${GREEN}✅${NC} $file ($SIZE)"
    else
        echo -e "  ${RED}❌${NC} $file (MISSING)"
        MISSING_REQUIRED=1
    fi
done

echo ""
echo -e "${BLUE}Important Files:${NC}"
MISSING_IMPORTANT=0
for file in "${IMPORTANT_FILES[@]}"; do
    if [ -f "$MODEL_PATH/$file" ]; then
        SIZE=$(du -h "$MODEL_PATH/$file" | cut -f1)
        echo -e "  ${GREEN}✅${NC} $file ($SIZE)"
    else
        echo -e "  ${YELLOW}⚠️${NC}  $file (not found)"
        MISSING_IMPORTANT=1
    fi
done

# Check for model weights
echo ""
echo -e "${BLUE}Model Weights:${NC}"
if [ -f "$MODEL_PATH/model.safetensors" ]; then
    SIZE=$(du -h "$MODEL_PATH/model.safetensors" | cut -f1)
    echo -e "  ${GREEN}✅${NC} model.safetensors ($SIZE)"
elif [ -f "$MODEL_PATH/pytorch_model.bin" ]; then
    SIZE=$(du -h "$MODEL_PATH/pytorch_model.bin" | cut -f1)
    echo -e "  ${GREEN}✅${NC} pytorch_model.bin ($SIZE)"
else
    # Check for sharded models
    SHARD_COUNT=$(ls "$MODEL_PATH"/model*.safetensors 2>/dev/null | wc -l)
    if [ "$SHARD_COUNT" -gt 0 ]; then
        TOTAL_SIZE=$(du -sh "$MODEL_PATH" | cut -f1)
        echo -e "  ${GREEN}✅${NC} Sharded model files ($SHARD_COUNT shards, total: $TOTAL_SIZE)"
    else
        echo -e "  ${RED}❌${NC} No model weights found!"
        MISSING_IMPORTANT=1
    fi
fi

# Calculate total size
TOTAL_SIZE=$(du -sh "$MODEL_PATH" | cut -f1)
echo ""
echo -e "${BLUE}Total Model Size:${NC} $TOTAL_SIZE"

# Summary
echo ""
if [ $MISSING_REQUIRED -eq 0 ]; then
    if [ $MISSING_IMPORTANT -eq 0 ]; then
        echo -e "${GREEN}✅ Model is complete and ready to use!${NC}"
        exit 0
    else
        echo -e "${YELLOW}⚠️  Model has required files but some important files are missing${NC}"
        echo "   Model may still work, but some features might not be available"
        exit 0
    fi
else
    echo -e "${RED}❌ Model is incomplete! Missing required files.${NC}"
    echo "   Please re-download the model from Colab"
    exit 1
fi
