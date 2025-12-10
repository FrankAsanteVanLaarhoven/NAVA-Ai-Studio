# NAVA-Instruct Fine-Tuning Guide

## Overview

This guide explains how to fine-tune a local 7B model on the NAVA-Instruct dataset to create a NAVA-fluent copilot.

## Dataset Structure

The dataset is in JSONL format with two files:
- `data/nava_instruct_train.jsonl` - Training examples (~80%)
- `data/nava_instruct_eval.jsonl` - Evaluation examples (~20%)

Each line is a JSON object:
```json
{
  "instruction": "<what the user wants>",
  "input": "<optional code snippet>",
  "output": "<what the assistant should reply>"
}
```

## Dataset Statistics

- **Total examples**: ~20-200 (expandable)
- **Split**: 70% prompt→NAVA, 30% code→explanation
- **Manifolds**: R^2, R^3, S^1, S^2, SE(2), SE(3)
- **Audience levels**: GCSE, A-level, undergrad, PhD

## Fine-Tuning Process

### 1. Choose Base Model

Recommended base models (7B-8B instruction-tuned):
- **Llama 3.1 8B Instruct** (recommended)
- **Mistral 7B Instruct**
- **Phi-3 Medium**
- **Qwen 2.5 7B Instruct**

### 2. Prepare Dataset

```bash
# Ensure dataset files exist
ls data/nava_instruct_train.jsonl
ls data/nava_instruct_eval.jsonl

# Optional: Generate more examples
node scripts/generate_nava_dataset.js
```

### 3. Tokenize Dataset

Convert JSONL to tokenized format for your training framework:

```python
# Example using HuggingFace
from datasets import load_dataset
from transformers import AutoTokenizer

tokenizer = AutoTokenizer.from_pretrained("meta-llama/Llama-3.1-8B-Instruct")

def format_instruction(example):
    if example["input"]:
        prompt = f"### Instruction:\n{example['instruction']}\n\n### Input:\n{example['input']}\n\n### Response:\n"
    else:
        prompt = f"### Instruction:\n{example['instruction']}\n\n### Response:\n"
    return {
        "text": prompt + example["output"] + tokenizer.eos_token
    }

dataset = load_dataset("json", data_files={
    "train": "data/nava_instruct_train.jsonl",
    "eval": "data/nava_instruct_eval.jsonl"
})

dataset = dataset.map(format_instruction)
```

### 4. Fine-Tune with LoRA/QLoRA

Using PEFT (Parameter-Efficient Fine-Tuning):

```python
from peft import LoraConfig, get_peft_model, TaskType
from transformers import AutoModelForCausalLM, TrainingArguments, Trainer

# Load model
model = AutoModelForCausalLM.from_pretrained(
    "meta-llama/Llama-3.1-8B-Instruct",
    load_in_8bit=True,  # For QLoRA
    device_map="auto"
)

# LoRA configuration
lora_config = LoraConfig(
    task_type=TaskType.CAUSAL_LM,
    r=16,
    lora_alpha=32,
    lora_dropout=0.1,
    target_modules=["q_proj", "v_proj", "k_proj", "o_proj"]
)

model = get_peft_model(model, lora_config)

# Training arguments
training_args = TrainingArguments(
    output_dir="./nava-instruct-7b",
    num_train_epochs=3,
    per_device_train_batch_size=4,
    gradient_accumulation_steps=4,
    learning_rate=2e-4,
    fp16=True,
    logging_steps=10,
    save_steps=100,
    evaluation_strategy="steps",
    eval_steps=100,
)

# Trainer
trainer = Trainer(
    model=model,
    args=training_args,
    train_dataset=dataset["train"],
    eval_dataset=dataset["eval"],
)

# Train
trainer.train()
```

### 5. Merge and Save

```python
# Merge LoRA weights
merged_model = model.merge_and_unload()

# Save
merged_model.save_pretrained("./nava-instruct-7b-merged")
tokenizer.save_pretrained("./nava-instruct-7b-merged")
```

## Serving the Model

### Option 1: vLLM

```python
from vllm import LLM, SamplingParams

llm = LLM(model="./nava-instruct-7b-merged")

sampling_params = SamplingParams(temperature=0.2, max_tokens=512)

prompt = "### Instruction:\nWrite NAVA code for a path from (0,0) to (5,5).\n\n### Response:\n"
outputs = llm.generate([prompt], sampling_params)
print(outputs[0].outputs[0].text)
```

### Option 2: llama.cpp

```bash
# Convert to GGUF
python convert_hf_to_gguf.py ./nava-instruct-7b-merged

# Serve
./llama-server -m nava-instruct-7b.gguf -p 8080
```

### Option 3: HuggingFace Transformers

```python
from transformers import AutoModelForCausalLM, AutoTokenizer

model = AutoModelForCausalLM.from_pretrained("./nava-instruct-7b-merged")
tokenizer = AutoTokenizer.from_pretrained("./nava-instruct-7b-merged")

prompt = "### Instruction:\nWrite NAVA code for a path from (0,0) to (5,5).\n\n### Response:\n"
inputs = tokenizer(prompt, return_tensors="pt")
outputs = model.generate(**inputs, max_length=512, temperature=0.2)
print(tokenizer.decode(outputs[0]))
```

## Integration with Router

Once the model is served, integrate it into your router:

```typescript
// In nava-local-backend.ts
class NAVALocalBackend implements LLMBackend {
  async chat(request: ChatRequest): Promise<ChatResponse> {
    // Format prompt using instruction format
    const formattedPrompt = this.formatInstructionPrompt(request.messages);
    
    // Call local model
    const response = await fetch('http://localhost:8080/v1/completions', {
      method: 'POST',
      body: JSON.stringify({
        prompt: formattedPrompt,
        temperature: 0.2,
        max_tokens: 512,
      }),
    });
    
    return this.parseResponse(response);
  }
  
  private formatInstructionPrompt(messages: ChatMessage[]): string {
    const lastMessage = messages[messages.length - 1];
    if (lastMessage.role === 'user') {
      return `### Instruction:\n${lastMessage.content}\n\n### Response:\n`;
    }
    return lastMessage.content;
  }
}
```

## Expected Results

After fine-tuning, the model should:
- ✅ Default to NAVA code generation
- ✅ Use correct NAVA syntax and idioms
- ✅ Explain code at appropriate levels
- ✅ Understand manifolds and cost functions
- ✅ Generate well-commented code for beginners

## Evaluation

Test the fine-tuned model on the eval set:

```python
# Evaluate
eval_results = trainer.evaluate()
print(f"Eval loss: {eval_results['eval_loss']}")

# Manual testing
test_prompts = [
    "Write NAVA code for a path from (0,0) to (5,5) with an obstacle.",
    "Explain this NAVA code: let M = euclidean_plane()...",
]

for prompt in test_prompts:
    output = generate(prompt)
    print(f"Prompt: {prompt}")
    print(f"Output: {output}\n")
```

## Next Steps

1. **Expand dataset**: Use `generate_nava_dataset.js` to create more examples
2. **Add code→code examples**: Refactoring, optimization, bug fixes
3. **Domain-specific examples**: DAAT/PDL/RT-shields patterns
4. **Multi-turn conversations**: Context-aware examples

## Resources

- [PEFT Documentation](https://huggingface.co/docs/peft)
- [LoRA Paper](https://arxiv.org/abs/2106.09685)
- [QLoRA Paper](https://arxiv.org/abs/2305.14314)
- [vLLM Documentation](https://docs.vllm.ai/)

