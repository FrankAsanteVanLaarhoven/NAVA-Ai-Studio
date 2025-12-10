# Colab Setup Guide for NAVA-Instruct Training

## Quick Start for Google Colab

### Step 1: Upload Dataset Files

1. Open the notebook in Colab: [NAVA_Instruct_Model_Training.ipynb](../LLM_Training_Notebook/NAVA_Instruct_Model_Training.ipynb)

2. Click the folder icon (📁) in the left sidebar

3. Upload these files to `/content/`:
   - `nava_instruct_train_enhanced.jsonl`
   - `nava_instruct_eval_enhanced.jsonl`

   **OR** upload the original files:
   - `nava_instruct_train.jsonl`
   - `nava_instruct_eval.jsonl`

### Step 2: Install Dependencies

Run this in a Colab cell:

```python
!pip install transformers datasets peft bitsandbytes accelerate torch
```

### Step 3: Run the Notebook

The notebook will:
- Auto-detect Colab environment
- Load dataset from `/content/` directory
- Generate synthetic data if files not found
- Train the model
- Save to `/content/models/`

### Step 4: Download Model

After training:

1. Navigate to `/content/models/` in the file browser
2. Right-click on `nava-instruct-7b-merged` folder
3. Click "Download"

**OR** use this command:

```python
!zip -r nava-instruct-7b.zip /content/models/nava-instruct-7b-merged
```

Then download the zip file.

## Alternative: Generate Dataset in Colab

If you don't have the dataset files, the notebook can generate synthetic data:

1. Run the "Generate Dataset in Colab" cell
2. It will create synthetic examples automatically
3. Files will be saved to `/content/` if possible

## Path Resolution

The notebook automatically detects:
- **Colab**: Uses `/content/` directory
- **Local**: Uses relative paths to NAVA Studio IDE

No manual path editing needed!

## Troubleshooting

### FileNotFoundError

**Solution**: Upload the dataset files to Colab first (see Step 1)

### Out of Memory

**Solution**: 
- Reduce `BATCH_SIZE` to 2 or 1
- Reduce `MAX_SEQ_LENGTH` to 512
- Use a smaller base model

### Model Download Fails

**Solution**: Use Colab's file browser to download instead of `!wget`

## Next Steps After Training

1. Download the merged model
2. Extract to your local machine
3. Serve the model:
   ```bash
   python scripts/serve_nava_model.py --model models/nava-instruct-7b-merged
   ```
4. Use in IDE: Select "🤖 NAVA Local (7B Fine-tuned)" in AI Panel

