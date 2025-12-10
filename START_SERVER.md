# 🚀 How to Start NAVA Model Server

## ⚠️ Current Issue
The server cannot start because the trained model file is not found.

## ✅ Solution: Train the Model First

### Step 1: Open Training Notebook
```
/Users/frankvanlaarhoven/Desktop/LLM_Training_Notebook/NAVA_Instruct_Model_Training.ipynb
```

### Step 2: Run All Cells
- Execute all cells in order
- This will train the NAVA-Instruct model
- Training takes 1-4 hours depending on your GPU

### Step 3: Model Will Be Saved To
```
./models/nava-llama-3.1-8b-instruct-merged
```

### Step 4: Start the Server
```bash
cd "/Users/frankvanlaarhoven/Desktop/NAVA Studio IDE"
./scripts/start_nava_model.sh
```

### Step 5: Verify It's Running
```bash
curl http://localhost:8080/health
```

Should return: `{"status":"ok"}`

## 🎯 Quick Start (Once Model Exists)

```bash
# Navigate to IDE directory
cd "/Users/frankvanlaarhoven/Desktop/NAVA Studio IDE"

# Start server
./scripts/start_nava_model.sh

# Or manually:
python3 scripts/serve_nava_model.py --backend huggingface --port 8080
```

## 🔍 Check Model Status

```bash
# Check if model exists
ls -la models/nava-llama-3.1-8b-instruct-merged

# Check if server is running
curl http://localhost:8080/health

# Check server logs
tail -f /tmp/nava_server_startup.log
```

## 💡 After Server Starts

1. Open AI Panel in IDE
2. Select **"🤖 NAVA Local (7B Fine-tuned)"** from dropdown
3. Start chatting!

The error message will disappear once the server is running! 🎉
