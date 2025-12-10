# NAVA Local Model Setup Guide

## ⚠️ "Failed to fetch" Error

If you see this error, it means the NAVA model server is not running.

## 🚀 Quick Start

### Step 1: Train the Model (First Time Only)

1. Open the training notebook:
   ```
   /Users/frankvanlaarhoven/Desktop/LLM_Training_Notebook/NAVA_Instruct_Model_Training.ipynb
   ```

2. Run all cells to train the model
   - This will create the fine-tuned model
   - Model will be saved to: `./models/nava-llama-3.1-8b-instruct-merged`

### Step 2: Start the Model Server

Once the model is trained, start the server:

```bash
cd "/Users/frankvanlaarhoven/Desktop/NAVA Studio IDE"
./scripts/start_nava_model.sh
```

Or manually:
```bash
python3 scripts/serve_nava_model.py --backend huggingface --port 8080
```

### Step 3: Verify It's Running

```bash
curl http://localhost:8080/health
```

Should return: `{"status":"ok"}`

### Step 4: Use in IDE

1. Open AI Panel in the IDE
2. Select **"🤖 NAVA Local (7B Fine-tuned)"** from model dropdown
3. Start chatting!

## 🔍 Troubleshooting

### Server won't start?

1. **Check if port 8080 is free:**
   ```bash
   lsof -i:8080
   ```

2. **Kill existing process if needed:**
   ```bash
   lsof -ti:8080 | xargs kill -9
   ```

3. **Check if model exists:**
   ```bash
   ls -la models/nava-llama-3.1-8b-instruct-merged
   ```

### Model not found?

- Train the model first using the notebook
- Or point to an existing model:
  ```bash
  python3 scripts/serve_nava_model.py \
    --model /path/to/your/model \
    --port 8080
  ```

## 📋 Status

- ✅ Error handling: Improved with helpful messages
- ⏳ Model server: Needs to be started
- ⏳ Model: Needs to be trained (or provided)

Once both are ready, the NAVA Local model will work perfectly!
