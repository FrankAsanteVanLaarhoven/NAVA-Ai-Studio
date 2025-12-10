#!/bin/bash

# NAVA Model Server Testing Script
# Tests the locally running NAVA-Instruct model

echo "🧪 Testing NAVA Model Server"
echo "============================"
echo ""

# Colors for output
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

BASE_URL="http://localhost:8080"

# Test 1: Health Check
echo "📋 Test 1: Health Check"
echo "----------------------"
if curl -s "${BASE_URL}/health" > /dev/null; then
    echo -e "${GREEN}✅ Server is running!${NC}"
    curl -s "${BASE_URL}/health" | python3 -m json.tool 2>/dev/null || curl -s "${BASE_URL}/health"
    echo ""
else
    echo -e "${RED}❌ Server is NOT running${NC}"
    echo "   Start it with: ./scripts/start_nava_model.sh"
    exit 1
fi

# Test 2: Simple Chat Completion
echo "📋 Test 2: Simple Chat Completion"
echo "---------------------------------"
echo "Sending: 'Hello, NAVA!'"
echo ""

RESPONSE=$(curl -s -X POST "${BASE_URL}/v1/chat/completions" \
  -H "Content-Type: application/json" \
  -d '{
    "messages": [
      {"role": "user", "content": "Hello, NAVA! Say hello back in one sentence."}
    ],
    "max_tokens": 50,
    "temperature": 0.7
  }')

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✅ Request successful!${NC}"
    echo ""
    echo "Response:"
    echo "$RESPONSE" | python3 -m json.tool 2>/dev/null || echo "$RESPONSE"
    echo ""
    
    # Extract just the text
    TEXT=$(echo "$RESPONSE" | python3 -c "import sys, json; data=json.load(sys.stdin); print(data.get('choices',[{}])[0].get('message',{}).get('content',''))" 2>/dev/null)
    if [ ! -z "$TEXT" ]; then
        echo -e "${GREEN}📝 Model said:${NC} $TEXT"
        echo ""
    fi
else
    echo -e "${RED}❌ Request failed${NC}"
    echo "$RESPONSE"
    exit 1
fi

# Test 3: NAVA Code Generation
echo "📋 Test 3: NAVA Code Generation"
echo "-------------------------------"
echo "Sending: 'Write NAVA code for a path from (0,0) to (5,5)'"
echo ""

RESPONSE=$(curl -s -X POST "${BASE_URL}/v1/chat/completions" \
  -H "Content-Type: application/json" \
  -d '{
    "messages": [
      {
        "role": "system",
        "content": "You are NAVA, a navigation calculus assistant. You write idiomatic NAVA code."
      },
      {
        "role": "user",
        "content": "Write NAVA code to create a navigation field from point (0,0) to point (5,5) on the Euclidean plane."
      }
    ],
    "max_tokens": 200,
    "temperature": 0.2
  }')

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✅ Request successful!${NC}"
    echo ""
    echo "Response:"
    echo "$RESPONSE" | python3 -m json.tool 2>/dev/null || echo "$RESPONSE"
    echo ""
    
    # Extract just the text
    TEXT=$(echo "$RESPONSE" | python3 -c "import sys, json; data=json.load(sys.stdin); print(data.get('choices',[{}])[0].get('message',{}).get('content',''))" 2>/dev/null)
    if [ ! -z "$TEXT" ]; then
        echo -e "${GREEN}📝 Model response:${NC}"
        echo "$TEXT"
        echo ""
    fi
else
    echo -e "${RED}❌ Request failed${NC}"
    echo "$RESPONSE"
fi

# Test 4: Check Usage Stats
echo "📋 Test 4: Usage Statistics"
echo "--------------------------"
RESPONSE=$(curl -s -X POST "${BASE_URL}/v1/chat/completions" \
  -H "Content-Type: application/json" \
  -d '{
    "messages": [{"role": "user", "content": "Hi"}],
    "max_tokens": 10
  }')

USAGE=$(echo "$RESPONSE" | python3 -c "import sys, json; data=json.load(sys.stdin); usage=data.get('usage',{}); print(f\"Input tokens: {usage.get('prompt_tokens',0)}, Output tokens: {usage.get('completion_tokens',0)}, Total: {usage.get('total_tokens',0)}\")" 2>/dev/null)

if [ ! -z "$USAGE" ]; then
    echo -e "${GREEN}✅ Usage stats:${NC} $USAGE"
else
    echo -e "${YELLOW}⚠️  Usage stats not available${NC}"
fi

echo ""
echo "============================"
echo -e "${GREEN}✅ All tests complete!${NC}"
echo ""
echo "💡 To use in the IDE:"
echo "   1. Open AI Panel"
echo "   2. Select '🤖 NAVA Local (7B Fine-tuned)'"
echo "   3. Start chatting!"
