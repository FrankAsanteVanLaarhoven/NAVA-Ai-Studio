# 🚀 Quick Guide: Download Model from Colab

## In Colab (https://colab.research.google.com/drive/1ff_W3NXnr-xnxYlDKtOelp1g6k97ODFJ)

### Step 1: Find the Model
1. Click **folder icon (📁)** in left sidebar
2. Go to `/content/`
3. Look for: `nava-llama-3.1-8b-instruct-merged` folder

### Step 2: Download (Choose One)

**Option A: Direct Download (Easiest)**
- Right-click `nava-llama-3.1-8b-instruct-merged` folder
- Click **Download**
- Wait (~15GB, 10-20 minutes)

**Option B: Zip First (Recommended for Large Files)**
```python
# Run this in Colab:
!zip -r nava-model.zip /content/nava-llama-3.1-8b-instruct-merged
```
- Then download `nava-model.zip` from file browser

**Option C: Google Drive (Best for Large Files)**
```python
# Run this in Colab:
!cp -r /content/nava-llama-3.1-8b-instruct-merged /content/drive/MyDrive/
```
- Then download from Google Drive

---

## On Your Desktop

### Step 1: Extract Model
Extract to:
```
/Users/frankvanlaarhoven/Desktop/NAVA Studio IDE/models/nava-llama-3.1-8b-instruct-merged/
```

### Step 2: Run Setup Script
```bash
cd "/Users/frankvanlaarhoven/Desktop/NAVA Studio IDE"
./setup_model_from_colab.sh
```

This will:
- ✅ Verify model files
- ✅ Create backup on Desktop
- ✅ Set up model path
- ✅ Show you how to start server

### Step 3: Start Server
```bash
./scripts/start_nava_model.sh
```

### Step 4: Test in IDE
1. Open AI Panel
2. Select "🤖 NAVA Local (7B Fine-tuned)"
3. Should show "Ready" ✅
4. Start chatting!

---

## What the Model Should Contain

After extraction, the folder should have:
- `config.json` ✅
- `tokenizer.json` or `tokenizer_config.json` ✅
- `model.safetensors` or `pytorch_model.bin` ✅
- `generation_config.json` ✅
- Other model files ✅

If any are missing, the model download may be incomplete.
