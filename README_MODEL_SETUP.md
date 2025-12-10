# 📚 NAVA Model Setup - Complete Documentation

## Quick Start

1. **Download from Colab** → See `QUICK_DOWNLOAD_GUIDE.md`
2. **Extract to Desktop** → `models/nava-llama-3.1-8b-instruct-merged/`
3. **Verify** → `./verify_model.sh`
4. **Setup** → `./setup_model_from_colab.sh`
5. **Start Server** → `./scripts/start_nava_model.sh`
6. **Test in IDE** → Select "🤖 NAVA Local (7B Fine-tuned)"

## Documentation Files

### 📥 Download Guides
- **`QUICK_DOWNLOAD_GUIDE.md`** - Quick reference for downloading from Colab
- **`DOWNLOAD_AND_SETUP_MODEL.md`** - Complete setup guide with troubleshooting

### 🔧 Setup Scripts
- **`setup_model_from_colab.sh`** - Automated setup script
  - Verifies model files
  - Creates Desktop backup
  - Configures model path
- **`verify_model.sh`** - Model verification script
  - Checks required files
  - Validates model structure
  - Reports missing files

### 📋 Checklists
- **`COMPLETE_SETUP_CHECKLIST.md`** - Step-by-step checklist
  - Phase-by-phase setup
  - Troubleshooting guide
  - Quick commands reference

### 🐍 Helper Scripts
- **`download_from_colab.py`** - Python helper for download instructions

## Model Location

After setup, model should be at:
```
/Users/frankvanlaarhoven/Desktop/NAVA Studio IDE/models/nava-llama-3.1-8b-instruct-merged/
```

Backup copy (created by setup script):
```
~/Desktop/nava-llama-3.1-8b-instruct-merged/
```

## Server Configuration

Default settings:
- **Port:** 8080
- **Backend:** HuggingFace Transformers
- **URL:** http://localhost:8080

Health check endpoint:
```bash
curl http://localhost:8080/health
```

## Common Issues

### Model Not Found
```bash
# Check if model exists
ls -la models/nava-llama-3.1-8b-instruct-merged/

# Verify files
./verify_model.sh
```

### Port Already in Use
```bash
# Check what's using port 8080
lsof -i:8080

# Kill process
lsof -ti:8080 | xargs kill -9
```

### Server Won't Start
```bash
# Check model path
export NAVA_MODEL_PATH="./models/nava-llama-3.1-8b-instruct-merged"

# Start manually
python3 scripts/serve_nava_model.py --backend huggingface --port 8080
```

## Next Steps After Setup

1. ✅ Model downloaded and verified
2. ✅ Server running on port 8080
3. ✅ IDE connected to NAVA Local model
4. 🎉 Start using NAVA-specific AI features!

## Support

If you encounter issues:
1. Run `./verify_model.sh` to check model integrity
2. Check server logs for error messages
3. Verify all files from checklist are present
4. Check `COMPLETE_SETUP_CHECKLIST.md` troubleshooting section
