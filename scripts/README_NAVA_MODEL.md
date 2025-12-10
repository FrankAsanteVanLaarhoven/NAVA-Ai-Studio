# NAVA-Instruct Model - Quick Start

## 🚀 Quick Start

### 1. Train the Model

Use the training notebook to fine-tune the model:
```bash
# Open in Jupyter/Colab
NAVA_Instruct_Model_Training.ipynb
```

After training, you'll have the merged model at:
- `./models/nava-llama-3.1-8b-instruct-merged/`

### 2. Start the Model Server

**Easiest way:**
```bash
./scripts/start_nava_model.sh
```

**Or with explicit path:**
```bash
./scripts/start_nava_model.sh ./models/nava-llama-3.1-8b-instruct-merged
```

**Or using Python:**
```bash
python3 scripts/serve_nava_model.py --backend huggingface
```

### 3. Use in IDE

The model automatically appears in the AI Panel as **"NAVA Local (7B Fine-tuned)"**. Just select it from the dropdown!

## 📋 Requirements

### For HuggingFace Backend (Default)
```bash
pip install transformers torch fastapi uvicorn
```

### For vLLM Backend (Faster, GPU required)
```bash
pip install vllm fastapi uvicorn
```

## 🔧 Configuration

### Environment Variables

```bash
export NAVA_MODEL_PATH=./models/nava-llama-3.1-8b-instruct-merged
export NAVA_MODEL_PORT=8080
export NAVA_MODEL_BACKEND=huggingface  # or vllm
```

### Auto-Detection

The server automatically searches for the model in:
1. `./models/nava-llama-3.1-8b-instruct-merged`
2. `./models/nava-instruct-7b-merged`
3. `../LLM_Training_Notebook/models/nava-llama-3.1-8b-instruct-merged`
4. `NAVA_MODEL_PATH` environment variable

## ✅ Verify It's Working

```bash
# Check health
curl http://localhost:8080/health

# Test generation
curl -X POST http://localhost:8080/v1/chat/completions \
  -H "Content-Type: application/json" \
  -d '{
    "messages": [
      {"role": "user", "content": "Plan a path from (0,0) to (5,5)"}
    ]
  }'
```

## 📚 Full Documentation

See `docs/NAVA_INSTRUCT_INTEGRATION.md` for complete details.

