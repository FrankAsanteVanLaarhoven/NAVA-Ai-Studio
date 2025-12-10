# NAVA-Instruct Fine-Tuning Guide

## Quick Start

### 1. Install Dependencies

```bash
pip install transformers datasets peft bitsandbytes accelerate torch
# For vLLM serving (optional, recommended)
pip install vllm fastapi uvicorn
```

### 2. Fine-Tune Model

```bash
python scripts/finetune_nava_instruct.py
```

This will:
- Load the NAVA-Instruct dataset
- Fine-tune Llama 3.1 8B with QLoRA
- Save model to `models/nava-instruct-7b`
- Create merged model at `models/nava-instruct-7b-merged`

### 3. Serve Model

**Option A: vLLM (Recommended - Fastest)**
```bash
python scripts/serve_nava_model.py \
  --model models/nava-instruct-7b-merged \
  --backend vllm \
  --port 8080
```

**Option B: HuggingFace (More Compatible)**
```bash
python scripts/serve_nava_model.py \
  --model models/nava-instruct-7b-merged \
  --backend huggingface \
  --port 8080
```

### 4. Test Fluency

```bash
python scripts/test_nava_fluency.py --server http://localhost:8080
```

### 5. Use in IDE

1. Start the model server (step 3)
2. Open NAVA Studio IDE
3. In AI Panel, select "NAVA Local (7B Fine-tuned)"
4. Ask for NAVA code - model will generate idiomatic NAVA!

## Files

- `finetune_nava_instruct.py` - Fine-tuning script
- `serve_nava_model.py` - Model serving script
- `test_nava_fluency.py` - Fluency testing script
- `../data/nava_instruct_train.jsonl` - Training dataset
- `../data/nava_instruct_eval.jsonl` - Eval dataset

## Model Output

- **LoRA Weights**: `models/nava-instruct-7b/` (for continued training)
- **Merged Model**: `models/nava-instruct-7b-merged/` (for serving)

## Requirements

- GPU with 8GB+ VRAM (for QLoRA)
- 20GB+ disk space (for model + dataset)
- Python 3.10+

## Troubleshooting

**Out of Memory**: Reduce `BATCH_SIZE` or `MAX_SEQ_LENGTH` in finetune script

**Model Not Loading**: Check that model path is correct and model files exist

**Server Not Starting**: Check port 8080 is available, try different port with `--port`

**Poor Results**: Expand dataset with `generate_nava_dataset.js`, train for more epochs

