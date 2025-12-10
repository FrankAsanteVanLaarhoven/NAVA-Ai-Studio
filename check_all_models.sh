#!/bin/bash

# Check Status of All Models
echo "📊 NAVA Studio IDE - Model Status Report"
echo "=========================================="
echo ""

# Colors
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# 1. NAVA Local Model
echo "🤖 NAVA Local (7B Fine-tuned)"
echo "----------------------------"
if curl -s http://localhost:8080/health > /dev/null 2>&1; then
    echo -e "${GREEN}✅ Status: RUNNING${NC}"
    echo "   URL: http://localhost:8080"
    echo "   Health: OK"
    echo -e "${GREEN}   ✅ Available to use${NC}"
else
    echo -e "${RED}❌ Status: NOT RUNNING${NC}"
    echo "   Reason: Server not started or model not found"
    echo -e "${YELLOW}   ⚠️  Not available${NC}"
    echo "   Fix: Train model and run ./scripts/start_nava_model.sh"
fi
echo ""

# 2. Check API Keys from environment
echo "🔑 API Keys Status"
echo "-----------------"
if [ -f .env ]; then
    if grep -q "VITE_GEMINI_API_KEY=" .env 2>/dev/null; then
        GEMINI_KEY=$(grep "VITE_GEMINI_API_KEY=" .env | cut -d'=' -f2)
        if [ ! -z "$GEMINI_KEY" ] && [ "$GEMINI_KEY" != "" ]; then
            echo -e "${GREEN}✅ Gemini API Key: SET${NC}"
        else
            echo -e "${YELLOW}⚠️  Gemini API Key: EMPTY${NC}"
        fi
    else
        echo -e "${RED}❌ Gemini API Key: NOT SET${NC}"
    fi
    
    if grep -q "VITE_OPENROUTER_API_KEY=" .env 2>/dev/null; then
        OR_KEY=$(grep "VITE_OPENROUTER_API_KEY=" .env | cut -d'=' -f2)
        if [ ! -z "$OR_KEY" ] && [ "$OR_KEY" != "" ]; then
            echo -e "${GREEN}✅ OpenRouter API Key: SET${NC}"
        else
            echo -e "${YELLOW}⚠️  OpenRouter API Key: EMPTY${NC}"
        fi
    else
        echo -e "${RED}❌ OpenRouter API Key: NOT SET${NC}"
    fi
    
    if grep -q "VITE_OPENAI_API_KEY=" .env 2>/dev/null; then
        OPENAI_KEY=$(grep "VITE_OPENAI_API_KEY=" .env | cut -d'=' -f2)
        if [ ! -z "$OPENAI_KEY" ] && [ "$OPENAI_KEY" != "" ]; then
            echo -e "${GREEN}✅ OpenAI API Key: SET${NC}"
        else
            echo -e "${YELLOW}⚠️  OpenAI API Key: EMPTY${NC}"
        fi
    else
        echo -e "${RED}❌ OpenAI API Key: NOT SET${NC}"
    fi
else
    echo -e "${YELLOW}⚠️  .env file not found${NC}"
    echo "   API keys may be set in browser localStorage or environment"
fi
echo ""

# 3. Available Models (based on config)
echo "📋 Available Models (from config)"
echo "---------------------------------"
echo -e "${BLUE}Built-in Models:${NC}"
echo "  • Gemini 1.5 Pro (requires Gemini API key)"
echo "  • Gemini 1.5 Flash (requires Gemini API key)"
echo "  • Gemini 2.0 Flash Experimental (requires Gemini API key)"
echo "  • OpenAI GPT-4o (requires OpenAI API key)"
echo "  • OpenAI GPT-4 Turbo (requires OpenAI API key)"
echo "  • OpenAI GPT-3.5 Turbo (requires OpenAI API key)"
echo "  • OpenRouter models (requires OpenRouter API key)"
echo "  • HuggingFace models (free, no key needed)"
echo "  • NAVA Local (7B Fine-tuned) - requires model server"
echo ""

# 4. Summary
echo "📊 Summary"
echo "---------"
echo ""
echo "✅ Available to use RIGHT NOW:"
if curl -s http://localhost:8080/health > /dev/null 2>&1; then
    echo "   • NAVA Local (7B Fine-tuned)"
fi
if [ -f .env ] && grep -q "VITE_GEMINI_API_KEY=" .env 2>/dev/null; then
    GEMINI_KEY=$(grep "VITE_GEMINI_API_KEY=" .env | cut -d'=' -f2)
    if [ ! -z "$GEMINI_KEY" ] && [ "$GEMINI_KEY" != "" ]; then
        echo "   • Gemini models (1.5 Pro, 1.5 Flash, 2.0 Flash)"
    fi
fi
if [ -f .env ] && grep -q "VITE_OPENROUTER_API_KEY=" .env 2>/dev/null; then
    OR_KEY=$(grep "VITE_OPENROUTER_API_KEY=" .env | cut -d'=' -f2)
    if [ ! -z "$OR_KEY" ] && [ "$OR_KEY" != "" ]; then
        echo "   • OpenRouter models"
    fi
fi
if [ -f .env ] && grep -q "VITE_OPENAI_API_KEY=" .env 2>/dev/null; then
    OPENAI_KEY=$(grep "VITE_OPENAI_API_KEY=" .env | cut -d'=' -f2)
    if [ ! -z "$OPENAI_KEY" ] && [ "$OPENAI_KEY" != "" ]; then
        echo "   • OpenAI models (GPT-4o, GPT-4 Turbo, GPT-3.5 Turbo)"
    fi
fi
echo "   • HuggingFace models (always available, free)"
echo ""
echo "❌ Not available:"
if ! curl -s http://localhost:8080/health > /dev/null 2>&1; then
    echo "   • NAVA Local (server not running)"
fi
if [ ! -f .env ] || ! grep -q "VITE_GEMINI_API_KEY=" .env 2>/dev/null || [ -z "$(grep "VITE_GEMINI_API_KEY=" .env | cut -d'=' -f2)" ]; then
    echo "   • Gemini models (no API key)"
fi
if [ ! -f .env ] || ! grep -q "VITE_OPENROUTER_API_KEY=" .env 2>/dev/null || [ -z "$(grep "VITE_OPENROUTER_API_KEY=" .env | cut -d'=' -f2)" ]; then
    echo "   • OpenRouter models (no API key)"
fi
if [ ! -f .env ] || ! grep -q "VITE_OPENAI_API_KEY=" .env 2>/dev/null || [ -z "$(grep "VITE_OPENAI_API_KEY=" .env | cut -d'=' -f2)" ]; then
    echo "   • OpenAI models (no API key)"
fi
echo ""
echo "💡 To add API keys:"
echo "   1. Open IDE → AI Panel → Settings (⚙️ icon)"
echo "   2. Add your API keys"
echo "   3. Or set in .env file: VITE_GEMINI_API_KEY=your_key"
echo ""
echo "=========================================="
