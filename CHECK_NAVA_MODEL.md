# NAVA Model Server - Quick Check

## ✅ Current Status

**Server:** http://localhost:8080  
**Health Check:** ✅ PASSED  
**Port Status:** ✅ IN USE

## 🚀 To Start/Restart the Model Server

### Option 1: Using Startup Script (Recommended)
```bash
cd "/Users/frankvanlaarhoven/Desktop/NAVA Studio IDE"
./scripts/start_nava_model.sh
```

### Option 2: Manual Start
```bash
cd "/Users/frankvanlaarhoven/Desktop/NAVA Studio IDE"
python3 scripts/serve_nava_model.py --backend huggingface
```

### Option 3: With Specific Model Path
```bash
python3 scripts/serve_nava_model.py \
  --model /path/to/nava-llama-3.1-8b-instruct-merged \
  --port 8080 \
  --backend huggingface
```

## 🔍 Verify It's Working

```bash
# Health check
curl http://localhost:8080/health

# Test generation
curl -X POST http://localhost:8080/v1/chat/completions \
  -H "Content-Type: application/json" \
  -d '{
    "messages": [
      {"role": "user", "content": "Hello, NAVA!"}
    ],
    "max_tokens": 50
  }'
```

## 📋 In the IDE

1. Open AI Panel
2. Select **"🤖 NAVA Local (7B Fine-tuned)"** from model dropdown
3. Start chatting!

## ⚠️ Troubleshooting

If the server is not running:
1. Check if port 8080 is free: `lsof -i:8080`
2. Kill existing process if needed: `lsof -ti:8080 | xargs kill -9`
3. Start the server: `./scripts/start_nava_model.sh`

