# NAVA Stack-Powered Training System

## Overview

This system uses the NAVA math stack (DSL + runtime + analyser) as a **data generator, grader, and teacher** to create a truly NAVA-fluent 7B model.

## Architecture

```
NAVA Stack
    ↓
┌─────────────────────────────────────┐
│ 1. Data Generator                   │
│    - Synthetic prompt→NAVA pairs    │
│    - Uses NAVA runtime as ground    │
│      truth                          │
└─────────────────────────────────────┘
    ↓
┌─────────────────────────────────────┐
│ 2. Checker / Grader                 │
│    - Validates generated code       │
│    - Creates bad→good pairs         │
│    - Uses runtime as oracle         │
└─────────────────────────────────────┘
    ↓
┌─────────────────────────────────────┐
│ 3. Explanation Generator            │
│    - Uses NAVA structure knowledge  │
│    - Generates multi-level explans  │
│    - GCSE → PhD                     │
└─────────────────────────────────────┘
    ↓
Enhanced NAVA-Instruct Dataset
    ↓
Fine-Tuned NAVA-7B Model
    ↓
Evaluation Harness (NAVA runtime as judge)
```

## Components

### 1. NAVA Data Generator (`nava_data_generator.py`)

**Purpose**: Generate synthetic training data using NAVA stack as ground truth

**How it works**:
- Samples random problems (manifolds, start/goal, obstacles, costs)
- Builds canonical NAVA code using NAVA stack patterns
- Generates natural language descriptions
- Creates perfectly consistent prompt→NAVA pairs

**Output**: Hundreds/thousands of training examples that reflect your geometry and naming conventions

### 2. NAVA Checker (`nava_checker.py`)

**Purpose**: Use NAVA runtime to check correctness and generate corrections

**How it works**:
- Checks if code parses correctly
- Validates structure (manifold, start, goal, field, path)
- Runs code through NAVA runtime
- Generates corrections for wrong code
- Creates bad→good training pairs

**Output**: High-value correction examples showing how to fix common mistakes

### 3. NAVA Explanation Generator (`nava_explanation_generator.py`)

**Purpose**: Use NAVA structure knowledge to generate explanations

**How it works**:
- Extracts concepts from code (manifold, field, obstacles, cost)
- Uses concept templates at different levels
- Generates GCSE/A-level/undergrad/PhD explanations
- Creates code→explanation training pairs

**Output**: Educational examples at multiple levels

### 4. DAAT/PDL Data Generator (`daat_pdl_data_generator.py`)

**Purpose**: Generate timing-aware NAVA code examples

**How it works**:
- Generates DAAT contract examples
- Creates PDL tier architecture examples
- Generates RT-shields contract examples
- Uses your standard patterns as templates

**Output**: Timing-aware training examples

### 5. Documentation Indexer (`nava_docs_indexer.py`)

**Purpose**: Build searchable index of NAVA documentation

**How it works**:
- Indexes all markdown files in nava-docs-suite
- Extracts sections, code examples, functions
- Creates searchable index
- Powers `fetch_nava_docs` tool

**Output**: Searchable documentation index

### 6. Evaluation Harness (`nava_eval_harness.py`)

**Purpose**: Evaluate model using NAVA runtime as judge

**How it works**:
- Generates code from prompts
- Checks if code parses
- Runs code through NAVA runtime
- Validates constraints (start/goal, obstacles)
- Reports success rates

**Output**: Formal evaluation metrics

## Enhanced Training Pipeline

### Step 1: Build Enhanced Dataset

```bash
# Generate synthetic data
python scripts/nava_data_generator.py

# Generate explanations
python scripts/nava_explanation_generator.py

# Generate DAAT/PDL examples
python scripts/daat_pdl_data_generator.py

# Index documentation
python scripts/nava_docs_indexer.py

# Combine everything
python scripts/build_nava_instruct_dataset.py
```

**Output**: `nava_instruct_train_enhanced.jsonl` with:
- Hand-written seed examples
- Synthetic prompt→NAVA (100+ examples)
- Code→explanation (multi-level)
- Bad→good corrections
- DAAT/PDL/RT-shields examples

### Step 2: Fine-Tune with Enhanced Dataset

```bash
python scripts/finetune_nava_instruct.py \
  --train_data data/nava_instruct_train_enhanced.jsonl \
  --eval_data data/nava_instruct_eval_enhanced.jsonl
```

### Step 3: Evaluate with NAVA Runtime

```bash
# Serve model
python scripts/serve_nava_model.py --model models/nava-instruct-7b-merged

# Evaluate
python scripts/nava_eval_harness.py \
  --server http://localhost:8080 \
  --benchmark data/nava_instruct_eval_enhanced.jsonl
```

**Metrics**:
- Parse success rate
- Runtime success rate
- Constraint satisfaction rate
- Overall pass rate

## Key Advantages

### 1. NAVA Stack as Ground Truth
- Generated code is always correct (uses your patterns)
- Consistent with your geometry and naming
- Reflects your actual runtime behavior

### 2. Runtime as Judge
- Not just "does it look right?"
- Actually runs code and checks results
- Validates paths, obstacles, constraints

### 3. Educational Alignment
- Explanations use your concept templates
- Matches your teaching style
- Consistent terminology

### 4. Timing-Aware
- DAAT/PDL/RT-shields examples from your patterns
- Model learns your timing contract style
- Ready for real robot deployment

## Integration with Training Notebook

The training notebook (`NAVA_Instruct_Model_Training.ipynb`) now includes:

1. **Dataset Building Cell**: Uses NAVA stack generators
2. **Quality Check Cell**: Uses NAVA checker before training
3. **Evaluation Cell**: Uses NAVA eval harness after training

## Result

You get a model that:

✅ **Generates correct NAVA code** (validated by runtime)
✅ **Uses your idioms** (learned from your patterns)
✅ **Understands timing** (DAAT/PDL/RT-shields)
✅ **Explains clearly** (at multiple levels)
✅ **Fixes mistakes** (learned from corrections)

This is how you go from:
- "LLM that can sort of write NAVA"
- To: "LLM that writes NAVA, the code runs, paths are valid, contracts verify"

## Next Steps

1. **Run data generators** to build enhanced dataset
2. **Fine-tune** on enhanced dataset
3. **Evaluate** with NAVA runtime
4. **Iterate** based on evaluation results
5. **Expand** dataset with more scenarios

The NAVA stack is now your training partner, not just a runtime!

