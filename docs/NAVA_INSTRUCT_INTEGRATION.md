# NAVA-Instruct Model Integration Guide

This guide explains how to integrate the fine-tuned NAVA-Instruct model into the NAVA Studio IDE.

## Overview

The NAVA-Instruct model is a fine-tuned Llama 3.1 8B Instruct model trained on the NAVA-Instruct dataset. It provides NAVA-fluent code generation and explanations.

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    NAVA Studio IDE                       │
│                                                          │
│  ┌──────────────┐      ┌──────────────┐                │
│  │  AI Panel    │──────│  LLM Hub     │                │
│  │              │      │              │                │
│  └──────────────┘      └──────┬───────┘                │
│                                │                        │
│                                ▼                        │
│                    ┌─────────────────────┐              │
│                    │ NAVA Local Backend │              │
│                    └──────────┬──────────┘              │
└───────────────────────────────┼─────────────────────────┘
                                │ HTTP (localhost:8080)
                                ▼
                    ┌───────────────────────┐
                    │  Model Server         │
                    │  (serve_nava_model.py)│
                    │                       │
                    │  ┌─────────────────┐ │
                    │  │ Fine-tuned Model │ │
                    │  │ (Merged LoRA)   │ │
                    │  └─────────────────┘ │
                    └───────────────────────┘
```

## Quick Start

### 1. Train the Model

Follow the training notebook (`NAVA_Instruct_Model_Training.ipynb`) to fine-tune the model. After training, you'll have:

- **LoRA Adapter**: `./models/nava-llama-3.1-8b-instruct/`
- **Merged Model**: `./models/nava-llama-3.1-8b-instruct-merged/`

### 2. Start the Model Server

**Option A: Using the startup script (recommended)**

```bash
# Auto-detect model path
./scripts/start_nava_model.sh

# Or specify model path
./scripts/start_nava_model.sh /path/to/nava-llama-3.1-8b-instruct-merged

# Or with custom port
./scripts/start_nava_model.sh /path/to/model 8080 huggingface
```

**Option B: Using Python directly**

```bash
# With auto-detection
python3 scripts/serve_nava_model.py --backend huggingface

# With explicit path
python3 scripts/serve_nava_model.py \
    --model ./models/nava-llama-3.1-8b-instruct-merged \
    --port 8080 \
    --backend huggingface
```

**Option C: Using environment variables**

```bash
export NAVA_MODEL_PATH=./models/nava-llama-3.1-8b-instruct-merged
export NAVA_MODEL_PORT=8080
export NAVA_MODEL_BACKEND=huggingface

./scripts/start_nava_model.sh
```

### 3. Verify Server is Running

```bash
# Check health endpoint
curl http://localhost:8080/health

# Test generation
curl -X POST http://localhost:8080/v1/chat/completions \
  -H "Content-Type: application/json" \
  -d '{
    "messages": [
      {"role": "user", "content": "Plan a path from (0,0) to (5,5)"}
    ],
    "temperature": 0.7,
    "max_tokens": 256
  }'
```

### 4. Use in IDE

The model will automatically appear in the AI Panel as **"NAVA Local (7B Fine-tuned)"**. Select it from the model dropdown to use it.

## Model Path Auto-Detection

The server automatically searches for the model in these locations (in order):

1. `./models/nava-llama-3.1-8b-instruct-merged`
2. `./models/nava-instruct-7b-merged`
3. `../LLM_Training_Notebook/models/nava-llama-3.1-8b-instruct-merged`
4. `../models/nava-llama-3.1-8b-instruct-merged`
5. Environment variable `NAVA_MODEL_PATH`

## Backend Options

### HuggingFace Backend (Default)

- **Pros**: More compatible, works on CPU, easier setup
- **Cons**: Slower inference (~10-20 tokens/sec)
- **Requirements**: `transformers`, `torch`, `fastapi`, `uvicorn`

```bash
pip install transformers torch fastapi uvicorn
```

### vLLM Backend (Faster)

- **Pros**: Much faster inference (~50-100 tokens/sec), better for production
- **Cons**: Requires GPU, more complex setup
- **Requirements**: `vllm`, `fastapi`, `uvicorn`

```bash
pip install vllm fastapi uvicorn
```

## API Endpoints

The model server provides OpenAI-compatible endpoints:

### POST `/v1/chat/completions`

Chat completion endpoint (recommended for fine-tuned models).

**Request:**
```json
{
  "messages": [
    {"role": "system", "content": "You are NAVA..."},
    {"role": "user", "content": "Plan a path from (0,0) to (5,5)"}
  ],
  "temperature": 0.7,
  "max_tokens": 512
}
```

**Response:**
```json
{
  "choices": [{
    "message": {
      "role": "assistant",
      "content": "Generated NAVA code..."
    },
    "finish_reason": "stop"
  }],
  "usage": {
    "prompt_tokens": 50,
    "completion_tokens": 200,
    "total_tokens": 250
  }
}
```

### POST `/v1/completions`

Legacy completion endpoint.

### GET `/health`

Health check endpoint.

## Integration Details

### Model Loading

The model is loaded using the `load_nava_model()` function from `scripts/load_nava_model.py`:

```python
from scripts.load_nava_model import load_nava_model

model, tokenizer = load_nava_model(
    model_path="./models/nava-llama-3.1-8b-instruct-merged",
    device_map="auto",
    torch_dtype=torch.bfloat16,
)
```

### Chat Template

The fine-tuned model uses the Llama chat template. The server automatically formats messages using `tokenizer.apply_chat_template()`:

```python
messages = [
    {"role": "system", "content": "You are NAVA..."},
    {"role": "user", "content": "User prompt"},
]
formatted = tokenizer.apply_chat_template(
    messages,
    tokenize=False,
    add_generation_prompt=True,
)
```

### Generation Parameters

Recommended settings for NAVA code generation:

- **Temperature**: 0.2-0.7 (lower for code, higher for explanations)
- **Max Tokens**: 256-512 (adjust based on complexity)
- **Top-p**: 0.95 (default)
- **Do Sample**: True (for diverse outputs)

## Troubleshooting

### Model Not Found

**Error**: `NAVA-Instruct model not found`

**Solution**:
1. Check model path exists: `ls -la ./models/nava-llama-3.1-8b-instruct-merged`
2. Verify `config.json` exists in model directory
3. Set `NAVA_MODEL_PATH` environment variable
4. Or specify `--model` argument explicitly

### Port Already in Use

**Error**: `Port 8080 is already in use`

**Solution**:
```bash
# Use different port
./scripts/start_nava_model.sh /path/to/model 8081

# Or kill existing process
lsof -ti:8080 | xargs kill
```

### CUDA Out of Memory

**Error**: `CUDA out of memory`

**Solution**:
1. Use CPU mode: Set `device_map="cpu"` in model loading
2. Use 4-bit quantization: Add `use_4bit=True` to model loading
3. Reduce batch size or max tokens
4. Use HuggingFace backend instead of vLLM

### Model Server Not Responding

**Error**: `NAVA Local backend error: Connection refused`

**Solution**:
1. Verify server is running: `curl http://localhost:8080/health`
2. Check firewall settings
3. Verify port matches: `VITE_NAVA_LOCAL_URL` in `.env`
4. Check server logs for errors

## Performance Tips

1. **Use GPU**: Significantly faster inference
2. **Use vLLM**: 3-5x faster than HuggingFace backend
3. **Batch Requests**: Process multiple requests together
4. **Cache Responses**: Cache common queries
5. **Optimize Temperature**: Lower temperature = faster generation

## Next Steps

- [ ] Train model on larger dataset
- [ ] Fine-tune hyperparameters
- [ ] Add evaluation metrics
- [ ] Deploy to production server
- [ ] Add model versioning
- [ ] Implement model caching

## Files

- `scripts/serve_nava_model.py` - Model server
- `scripts/load_nava_model.py` - Model loader utility
- `scripts/start_nava_model.sh` - Startup script
- `src/services/llm-hub/backends/nava-local-backend.ts` - IDE integration
- `NAVA_Instruct_Model_Training.ipynb` - Training notebook

## Support

For issues or questions:
1. Check server logs
2. Verify model path and format
3. Test with curl commands
4. Check IDE console for errors

