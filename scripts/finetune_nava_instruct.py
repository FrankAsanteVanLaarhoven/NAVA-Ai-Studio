#!/usr/bin/env python3
"""
NAVA-Instruct Fine-Tuning Script
Fine-tunes a 7B model on the NAVA-Instruct dataset using LoRA/QLoRA
"""

import json
import os
from pathlib import Path
from datasets import load_dataset, Dataset
from transformers import (
    AutoModelForCausalLM,
    AutoTokenizer,
    TrainingArguments,
    Trainer,
    DataCollatorForLanguageModeling
)
from peft import LoraConfig, get_peft_model, TaskType, prepare_model_for_kbit_training
from bitsandbytes import BitsAndBytesConfig
import torch

# Configuration
BASE_MODEL = "meta-llama/Llama-3.1-8B-Instruct"  # or "mistralai/Mistral-7B-Instruct-v0.2"
DATASET_TRAIN = "data/nava_instruct_train.jsonl"
DATASET_EVAL = "data/nava_instruct_eval.jsonl"
OUTPUT_DIR = "models/nava-instruct-7b"
MAX_SEQ_LENGTH = 1024
BATCH_SIZE = 4
GRADIENT_ACCUMULATION_STEPS = 4
LEARNING_RATE = 2e-4
NUM_EPOCHS = 3
LORA_R = 16
LORA_ALPHA = 32
LORA_DROPOUT = 0.1

def load_jsonl(file_path):
    """Load JSONL file into list of dicts"""
    data = []
    with open(file_path, 'r', encoding='utf-8') as f:
        for line in f:
            if line.strip():
                data.append(json.loads(line))
    return data

def format_instruction(example, tokenizer):
    """Format instruction dataset into prompt format"""
    if example.get("input"):
        prompt = f"### Instruction:\n{example['instruction']}\n\n### Input:\n{example['input']}\n\n### Response:\n"
    else:
        prompt = f"### Instruction:\n{example['instruction']}\n\n### Response:\n"
    
    full_text = prompt + example["output"] + tokenizer.eos_token
    
    return {"text": full_text}

def tokenize_function(examples, tokenizer, max_length):
    """Tokenize the formatted text"""
    return tokenizer(
        examples["text"],
        truncation=True,
        max_length=max_length,
        padding="max_length",
    )

def main():
    print("🚀 Starting NAVA-Instruct Fine-Tuning")
    print(f"📦 Base Model: {BASE_MODEL}")
    print(f"📊 Training Data: {DATASET_TRAIN}")
    print(f"📊 Eval Data: {DATASET_EVAL}")
    print(f"💾 Output Directory: {OUTPUT_DIR}")
    
    # Load tokenizer
    print("\n📥 Loading tokenizer...")
    tokenizer = AutoTokenizer.from_pretrained(BASE_MODEL)
    if tokenizer.pad_token is None:
        tokenizer.pad_token = tokenizer.eos_token
    tokenizer.padding_side = "right"
    
    # Load dataset
    print("\n📥 Loading dataset...")
    train_data = load_jsonl(DATASET_TRAIN)
    eval_data = load_jsonl(DATASET_EVAL)
    
    print(f"✅ Loaded {len(train_data)} training examples")
    print(f"✅ Loaded {len(eval_data)} eval examples")
    
    # Format dataset
    print("\n🔄 Formatting dataset...")
    train_dataset = Dataset.from_list(train_data)
    eval_dataset = Dataset.from_list(eval_data)
    
    train_dataset = train_dataset.map(
        lambda x: format_instruction(x, tokenizer),
        remove_columns=train_dataset.column_names
    )
    eval_dataset = eval_dataset.map(
        lambda x: format_instruction(x, tokenizer),
        remove_columns=eval_dataset.column_names
    )
    
    # Tokenize
    print("\n🔄 Tokenizing dataset...")
    train_dataset = train_dataset.map(
        lambda x: tokenize_function(x, tokenizer, MAX_SEQ_LENGTH),
        batched=True,
        remove_columns=["text"]
    )
    eval_dataset = eval_dataset.map(
        lambda x: tokenize_function(x, tokenizer, MAX_SEQ_LENGTH),
        batched=True,
        remove_columns=["text"]
    )
    
    # Load model with QLoRA
    print("\n📥 Loading model with QLoRA...")
    bnb_config = BitsAndBytesConfig(
        load_in_4bit=True,
        bnb_4bit_quant_type="nf4",
        bnb_4bit_compute_dtype=torch.float16,
        bnb_4bit_use_double_quant=True,
    )
    
    model = AutoModelForCausalLM.from_pretrained(
        BASE_MODEL,
        quantization_config=bnb_config,
        device_map="auto",
        trust_remote_code=True,
    )
    
    # Prepare for LoRA
    model = prepare_model_for_kbit_training(model)
    
    # LoRA configuration
    lora_config = LoraConfig(
        task_type=TaskType.CAUSAL_LM,
        r=LORA_R,
        lora_alpha=LORA_ALPHA,
        lora_dropout=LORA_DROPOUT,
        target_modules=["q_proj", "v_proj", "k_proj", "o_proj", "gate_proj", "up_proj", "down_proj"],
        bias="none",
    )
    
    model = get_peft_model(model, lora_config)
    model.print_trainable_parameters()
    
    # Data collator
    data_collator = DataCollatorForLanguageModeling(
        tokenizer=tokenizer,
        mlm=False,
    )
    
    # Training arguments
    training_args = TrainingArguments(
        output_dir=OUTPUT_DIR,
        num_train_epochs=NUM_EPOCHS,
        per_device_train_batch_size=BATCH_SIZE,
        per_device_eval_batch_size=BATCH_SIZE,
        gradient_accumulation_steps=GRADIENT_ACCUMULATION_STEPS,
        learning_rate=LEARNING_RATE,
        fp16=True,
        logging_steps=10,
        save_steps=100,
        evaluation_strategy="steps",
        eval_steps=50,
        save_total_limit=3,
        load_best_model_at_end=True,
        metric_for_best_model="eval_loss",
        greater_is_better=False,
        warmup_steps=50,
        lr_scheduler_type="cosine",
        report_to="tensorboard",
    )
    
    # Trainer
    print("\n🏋️ Setting up trainer...")
    trainer = Trainer(
        model=model,
        args=training_args,
        train_dataset=train_dataset,
        eval_dataset=eval_dataset,
        data_collator=data_collator,
    )
    
    # Train
    print("\n🚀 Starting training...")
    trainer.train()
    
    # Save
    print("\n💾 Saving model...")
    trainer.save_model()
    tokenizer.save_pretrained(OUTPUT_DIR)
    
    # Save training info
    training_info = {
        "base_model": BASE_MODEL,
        "dataset_train": DATASET_TRAIN,
        "dataset_eval": DATASET_EVAL,
        "num_train_examples": len(train_data),
        "num_eval_examples": len(eval_data),
        "max_seq_length": MAX_SEQ_LENGTH,
        "batch_size": BATCH_SIZE,
        "learning_rate": LEARNING_RATE,
        "num_epochs": NUM_EPOCHS,
        "lora_r": LORA_R,
        "lora_alpha": LORA_ALPHA,
    }
    
    with open(f"{OUTPUT_DIR}/training_info.json", "w") as f:
        json.dump(training_info, f, indent=2)
    
    print(f"\n✅ Fine-tuning complete! Model saved to {OUTPUT_DIR}")
    print("\n📊 Final evaluation:")
    eval_results = trainer.evaluate()
    print(f"Eval Loss: {eval_results['eval_loss']:.4f}")
    
    # Merge LoRA weights (optional)
    print("\n🔄 Merging LoRA weights...")
    merged_model = model.merge_and_unload()
    merged_output_dir = f"{OUTPUT_DIR}-merged"
    merged_model.save_pretrained(merged_output_dir)
    tokenizer.save_pretrained(merged_output_dir)
    print(f"✅ Merged model saved to {merged_output_dir}")

if __name__ == "__main__":
    main()

