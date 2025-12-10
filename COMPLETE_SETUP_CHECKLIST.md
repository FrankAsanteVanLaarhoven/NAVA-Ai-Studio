# ✅ Complete Setup Checklist

## Phase 1: Download from Colab

- [ ] Open Colab notebook: https://colab.research.google.com/drive/1ff_W3NXnr-xnxYlDKtOelp1g6k97ODFJ
- [ ] Click folder icon (📁) in left sidebar
- [ ] Navigate to `/content/`
- [ ] Find `nava-llama-3.1-8b-instruct-merged` folder
- [ ] Choose download method:
  - [ ] Option A: Right-click → Download (direct)
  - [ ] Option B: Zip first (`!zip -r nava-model.zip /content/nava-llama-3.1-8b-instruct-merged`)
  - [ ] Option C: Save to Google Drive first (`!cp -r /content/nava-llama-3.1-8b-instruct-merged /content/drive/MyDrive/`)
- [ ] Wait for download to complete (~15GB, 10-20 minutes)

## Phase 2: Extract on Desktop

- [ ] Extract downloaded folder/zip
- [ ] Place in: `/Users/frankvanlaarhoven/Desktop/NAVA Studio IDE/models/`
- [ ] Final path: `models/nava-llama-3.1-8b-instruct-merged/`
- [ ] Verify folder structure looks correct

## Phase 3: Verify Model

- [ ] Run: `./verify_model.sh`
- [ ] Check all required files are present:
  - [ ] `config.json` ✅
  - [ ] `generation_config.json` ✅
  - [ ] `tokenizer.json` or `tokenizer_config.json` ✅
  - [ ] `model.safetensors` or `pytorch_model.bin` ✅
- [ ] Verify model size is reasonable (~15GB)

## Phase 4: Setup

- [ ] Run: `./setup_model_from_colab.sh`
- [ ] Script will:
  - [ ] Verify model files
  - [ ] Create backup on Desktop
  - [ ] Configure model path
  - [ ] Show next steps

## Phase 5: Start Server

- [ ] Run: `./scripts/start_nava_model.sh`
- [ ] Wait for "Server running on http://localhost:8080"
- [ ] Verify health: `curl http://localhost:8080/health`
- [ ] Should return: `{"status":"healthy","model":"..."}`

## Phase 6: Test in IDE

- [ ] Open NAVA Studio IDE
- [ ] Open AI Panel
- [ ] Select "🤖 NAVA Local (7B Fine-tuned)"
- [ ] Check status shows "Ready" ✅
- [ ] Send a test message
- [ ] Verify you get a real AI response (not error)

## Troubleshooting

### Model Not Found
- [ ] Check path: `ls -la models/nava-llama-3.1-8b-instruct-merged/`
- [ ] Verify `config.json` exists
- [ ] Run: `./verify_model.sh` for detailed check

### Server Won't Start
- [ ] Check port 8080 is free: `lsof -i:8080`
- [ ] Kill existing process if needed: `lsof -ti:8080 | xargs kill -9`
- [ ] Check model path is correct
- [ ] Check you have enough RAM (~8GB free)

### Model Not Working in IDE
- [ ] Verify server is running: `curl http://localhost:8080/health`
- [ ] Check browser console for errors
- [ ] Verify backend is available in IDE
- [ ] Try restarting IDE

## Quick Commands Reference

```bash
# Verify model
./verify_model.sh

# Setup model
./setup_model_from_colab.sh

# Start server
./scripts/start_nava_model.sh

# Check server health
curl http://localhost:8080/health

# Check if port is in use
lsof -i:8080

# Kill process on port 8080
lsof -ti:8080 | xargs kill -9
```

## Expected File Structure

```
models/nava-llama-3.1-8b-instruct-merged/
├── config.json                    ✅ Required
├── generation_config.json         ✅ Required
├── tokenizer.json                 ✅ Important
├── tokenizer_config.json          ✅ Important
├── model.safetensors              ✅ Important (or pytorch_model.bin)
├── special_tokens_map.json        Optional
├── tokenizer.model               Optional
└── ... (other model files)
```

## Success Indicators

✅ Model verification passes  
✅ Setup script completes without errors  
✅ Server starts and responds to health check  
✅ IDE shows "Ready" status for NAVA Local model  
✅ Test message gets real AI response  

---

**Need Help?**
- Check `QUICK_DOWNLOAD_GUIDE.md` for download steps
- Check `DOWNLOAD_AND_SETUP_MODEL.md` for detailed setup
- Run `./verify_model.sh` to diagnose issues
- Check server logs for error messages
