# 🧪 Quick Testing Commands for NAVA Model

## Prerequisites
Make sure the model server is running:
```bash
./scripts/start_nava_model.sh
```

## Test Commands

### 1. Health Check (Simplest)
```bash
curl http://localhost:8080/health
```
**Expected:** `{"status":"ok"}`

---

### 2. Simple Chat Test
```bash
curl -X POST http://localhost:8080/v1/chat/completions \
  -H "Content-Type: application/json" \
  -d '{
    "messages": [
      {"role": "user", "content": "Hello, NAVA! Say hello back."}
    ],
    "max_tokens": 50,
    "temperature": 0.7
  }'
```

---

### 3. NAVA Code Generation Test
```bash
curl -X POST http://localhost:8080/v1/chat/completions \
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
  }'
```

---

### 4. Pretty Print Response (with Python)
```bash
curl -s -X POST http://localhost:8080/v1/chat/completions \
  -H "Content-Type: application/json" \
  -d '{
    "messages": [{"role": "user", "content": "Hello!"}],
    "max_tokens": 50
  }' | python3 -m json.tool
```

---

### 5. Extract Just the Text Response
```bash
curl -s -X POST http://localhost:8080/v1/chat/completions \
  -H "Content-Type: application/json" \
  -d '{
    "messages": [{"role": "user", "content": "Hello!"}],
    "max_tokens": 50
  }' | python3 -c "import sys, json; data=json.load(sys.stdin); print(data['choices'][0]['message']['content'])"
```

---

### 6. Full Test Suite
Run the comprehensive test script:
```bash
./test_nava_model.sh
```

This will test:
- ✅ Health check
- ✅ Simple chat completion
- ✅ NAVA code generation
- ✅ Usage statistics

---

## Expected Response Format

```json
{
  "choices": [
    {
      "message": {
        "role": "assistant",
        "content": "Hello! I'm NAVA..."
      },
      "finish_reason": "stop"
    }
  ],
  "usage": {
    "prompt_tokens": 10,
    "completion_tokens": 15,
    "total_tokens": 25
  }
}
```

---

## Troubleshooting

### Server not running?
```bash
# Check if running
curl http://localhost:8080/health

# Start it
./scripts/start_nava_model.sh
```

### Connection refused?
- Make sure port 8080 is free: `lsof -i:8080`
- Check server logs for errors

### Model not found?
- Train the model first using `NAVA_Instruct_Model_Training.ipynb`
- Or provide model path: `python3 scripts/serve_nava_model.py --model /path/to/model`
