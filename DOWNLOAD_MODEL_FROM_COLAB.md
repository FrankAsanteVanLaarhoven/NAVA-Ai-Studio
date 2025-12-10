# 📥 Download NAVA Model from Colab

## Step 1: Locate Model in Colab

The model should be saved in one of these locations in Colab:
- `/content/nava-llama-3.1-8b-instruct-merged/` (merged model - recommended)
- `/content/nava-llama-3.1-8b-instruct/` (LoRA adapter only)

## Step 2: Download from Colab

### Option A: Download via File Browser (Easiest)
1. In Colab, click folder icon (📁) in left sidebar
2. Navigate to `/content/`
3. Find `nava-llama-3.1-8b-instruct-merged` folder
4. Right-click → Download
5. Wait for download (may take 10-20 minutes, model is ~15GB)

### Option B: Create Zip and Download
```python
# In Colab, run this cell:
!zip -r nava-model.zip /content/nava-llama-3.1-8b-instruct-merged
# Then download nava-model.zip from file browser
```

### Option C: Save to Google Drive
```python
# In Colab, run this cell:
!cp -r /content/nava-llama-3.1-8b-instruct-merged /content/drive/MyDrive/
# Then download from Google Drive
```

## Step 3: Extract on Desktop

1. Extract the downloaded folder to:
   `/Users/frankvanlaarhoven/Desktop/NAVA Studio IDE/models/`

2. Final path should be:
   `/Users/frankvanlaarhoven/Desktop/NAVA Studio IDE/models/nava-llama-3.1-8b-instruct-merged/`

## Step 4: Verify Model Files

The folder should contain:
- `config.json`
- `tokenizer.json` or `tokenizer_config.json`
- `model.safetensors` or `pytorch_model.bin`
- `generation_config.json`
- Other model files

## Step 5: Start Server

```bash
cd "/Users/frankvanlaarhoven/Desktop/NAVA Studio IDE"
./scripts/start_nava_model.sh
```

Or set custom path:
```bash
export NAVA_MODEL_PATH="/Users/frankvanlaarhoven/Desktop/NAVA Studio IDE/models/nava-llama-3.1-8b-instruct-merged"
python3 scripts/serve_nava_model.py --backend huggingface --port 8080
```
