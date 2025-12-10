# 📥 Download NAVA Model from Colab & Setup on Desktop

## Step 1: Download Model from Colab

The model is saved at: `/content/nava-llama-3.1-8b-instruct-merged/`

### Option A: Download via File Browser (Recommended)
1. In Colab, click **folder icon (📁)** in left sidebar
2. Navigate to `/content/`
3. Find `nava-llama-3.1-8b-instruct-merged` folder
4. **Right-click** → **Download**
5. Wait for download (model is ~15GB, may take 10-20 minutes)

### Option B: Create Zip First (Faster Download)
```python
# In Colab, run this cell:
!zip -r nava-model.zip /content/nava-llama-3.1-8b-instruct-merged
# Then download nava-model.zip from file browser
```

### Option C: Save to Google Drive First
```python
# In Colab, run this cell:
!cp -r /content/nava-llama-3.1-8b-instruct-merged /content/drive/MyDrive/
# Then download from Google Drive (easier for large files)
```

---

## Step 2: Extract on Desktop

1. **Extract** the downloaded folder (or zip) to:
   ```
   /Users/frankvanlaarhoven/Desktop/NAVA Studio IDE/models/
   ```

2. **Final path should be:**
   ```
   /Users/frankvanlaarhoven/Desktop/NAVA Studio IDE/models/nava-llama-3.1-8b-instruct-merged/
   ```

3. **Verify** the folder contains:
   - `config.json` ✅
   - `tokenizer.json` or `tokenizer_config.json` ✅
   - `model.safetensors` or `pytorch_model.bin` ✅
   - `generation_config.json` ✅
   - Other model files ✅

---

## Step 3: Start the Server

### Automatic (Recommended)
```bash
cd "/Users/frankvanlaarhoven/Desktop/NAVA Studio IDE"
./scripts/start_nava_model.sh
```

The script will auto-detect the model at:
- `./models/nava-llama-3.1-8b-instruct-merged/`

### Manual (If auto-detect fails)
```bash
cd "/Users/frankvanlaarhoven/Desktop/NAVA Studio IDE"
export NAVA_MODEL_PATH="./models/nava-llama-3.1-8b-instruct-merged"
python3 scripts/serve_nava_model.py --backend huggingface --port 8080
```

---

## Step 4: Verify It's Working

### Check Server Health
```bash
curl http://localhost:8080/health
# Should return: {"status":"healthy","model":"..."}
```

### Test in IDE
1. Open AI Panel
2. Select "🤖 NAVA Local (7B Fine-tuned)"
3. Should show "Ready" status
4. Start chatting!

---

## Troubleshooting

### Model Not Found
If server says "model not found":
1. Check path: `ls -la models/nava-llama-3.1-8b-instruct-merged/`
2. Verify `config.json` exists: `ls models/nava-llama-3.1-8b-instruct-merged/config.json`
3. Set path manually: `export NAVA_MODEL_PATH="/full/path/to/model"`

### Port Already in Use
If port 8080 is busy:
```bash
# Kill existing process
lsof -ti:8080 | xargs kill -9

# Or use different port
python3 scripts/serve_nava_model.py --port 8081
```

### Model Loading Errors
- Check you have enough RAM (model needs ~8GB)
- Check GPU availability if using vLLM
- Use `--backend huggingface` for CPU compatibility
