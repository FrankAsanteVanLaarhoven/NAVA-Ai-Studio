# -*- coding: utf-8 -*-
"""
NAVA-Instruct Model Loader
Loads the fine-tuned NAVA-Instruct model for inference
"""

import os
import torch
from pathlib import Path
from transformers import AutoTokenizer, AutoModelForCausalLM
from typing import Optional, Tuple

def find_model_path() -> Optional[str]:
    """Find the fine-tuned model in common locations"""
    possible_paths = [
        # Local paths
        "./models/nava-llama-3.1-8b-instruct-merged",
        "./models/nava-instruct-7b-merged",
        "../LLM_Training_Notebook/models/nava-llama-3.1-8b-instruct-merged",
        "../models/nava-llama-3.1-8b-instruct-merged",
        # Colab paths
        "/content/nava-llama-3.1-8b-instruct-merged",
        "/content/models/nava-instruct-7b-merged",
        # Kaggle paths
        "/kaggle/working/nava-llama-3.1-8b-instruct-merged",
        # Environment variable
        os.getenv("NAVA_MODEL_PATH"),
    ]
    
    for path in possible_paths:
        if path and os.path.exists(path):
            # Check if it's a valid model directory
            if os.path.exists(os.path.join(path, "config.json")):
                return path
    
    return None

def load_nava_model(
    model_path: Optional[str] = None,
    device_map: str = "auto",
    torch_dtype: Optional[torch.dtype] = None,
    use_4bit: bool = False,
    trust_remote_code: bool = True,
) -> Tuple[AutoModelForCausalLM, AutoTokenizer]:
    """
    Load the fine-tuned NAVA-Instruct model and tokenizer
    
    Args:
        model_path: Path to the model. If None, will search common locations
        device_map: Device mapping strategy ("auto", "cpu", "cuda:0", etc.)
        torch_dtype: Data type for model (torch.float16, torch.bfloat16, etc.)
        use_4bit: Whether to use 4-bit quantization
        trust_remote_code: Whether to trust remote code in model
    
    Returns:
        Tuple of (model, tokenizer)
    """
    # Find model path if not provided
    if model_path is None:
        model_path = find_model_path()
        if model_path is None:
            raise FileNotFoundError(
                "NAVA-Instruct model not found. Please specify model_path or ensure model is in one of:\n"
                "  - ./models/nava-llama-3.1-8b-instruct-merged\n"
                "  - ./models/nava-instruct-7b-merged\n"
                "  - Set NAVA_MODEL_PATH environment variable"
            )
    
    print(f"📥 Loading NAVA-Instruct model from: {model_path}")
    
    # Determine dtype
    if torch_dtype is None:
        if torch.cuda.is_available():
            torch_dtype = torch.bfloat16
        else:
            torch_dtype = torch.float32
    
    # Load tokenizer
    print("   Loading tokenizer...")
    tokenizer = AutoTokenizer.from_pretrained(
        model_path,
        trust_remote_code=trust_remote_code,
    )
    
    # Ensure pad token is set
    if tokenizer.pad_token is None:
        tokenizer.pad_token = tokenizer.eos_token
    tokenizer.padding_side = "right"
    
    # Load model
    print("   Loading model...")
    model_kwargs = {
        "device_map": device_map,
        "torch_dtype": torch_dtype,
        "trust_remote_code": trust_remote_code,
    }
    
    # Add quantization if requested
    if use_4bit:
        from transformers import BitsAndBytesConfig
        model_kwargs["quantization_config"] = BitsAndBytesConfig(
            load_in_4bit=True,
            bnb_4bit_quant_type="nf4",
            bnb_4bit_compute_dtype=torch_dtype,
            bnb_4bit_use_double_quant=True,
        )
    
    model = AutoModelForCausalLM.from_pretrained(
        model_path,
        **model_kwargs
    )
    
    # Disable cache for inference (saves memory)
    model.config.use_cache = False
    
    print(f"✅ Model loaded successfully!")
    if torch.cuda.is_available():
        print(f"   Device: {next(model.parameters()).device}")
        print(f"   Memory: {torch.cuda.memory_allocated() / 1e9:.2f} GB allocated")
    
    return model, tokenizer

def generate_text(
    model: AutoModelForCausalLM,
    tokenizer: AutoTokenizer,
    prompt: str,
    max_new_tokens: int = 256,
    temperature: float = 0.7,
    use_chat_template: bool = True,
) -> str:
    """
    Generate text from the fine-tuned model
    
    Args:
        model: Loaded model
        tokenizer: Loaded tokenizer
        prompt: Input prompt
        max_new_tokens: Maximum tokens to generate
        temperature: Sampling temperature
        use_chat_template: Whether to use tokenizer's chat template
    
    Returns:
        Generated text
    """
    model.eval()
    
    # Format prompt using chat template if available
    if use_chat_template and hasattr(tokenizer, 'apply_chat_template'):
        messages = [
            {
                "role": "system",
                "content": "You are NAVA, a navigation calculus assistant. You speak in clear, precise mathematical language, and you write idiomatic NAVA code."
            },
            {"role": "user", "content": prompt},
        ]
        formatted_prompt = tokenizer.apply_chat_template(
            messages,
            tokenize=False,
            add_generation_prompt=True,
        )
    else:
        formatted_prompt = prompt
    
    # Tokenize
    inputs = tokenizer(formatted_prompt, return_tensors="pt").to(model.device)
    
    # Generate
    with torch.no_grad():
        if torch.cuda.is_available():
            with torch.autocast(device_type="cuda", dtype=torch.bfloat16):
                outputs = model.generate(
                    **inputs,
                    max_new_tokens=max_new_tokens,
                    temperature=temperature,
                    do_sample=True,
                    top_p=0.95,
                    pad_token_id=tokenizer.pad_token_id,
                )
        else:
            outputs = model.generate(
                **inputs,
                max_new_tokens=max_new_tokens,
                temperature=temperature,
                do_sample=True,
                top_p=0.95,
                pad_token_id=tokenizer.pad_token_id,
            )
    
    # Decode
    generated_text = tokenizer.decode(outputs[0], skip_special_tokens=True)
    
    # Extract only the generated part
    if formatted_prompt in generated_text:
        generated_text = generated_text.split(formatted_prompt)[-1].strip()
    
    return generated_text

if __name__ == "__main__":
    # Test loading
    try:
        model, tokenizer = load_nava_model()
        print("\n🧪 Testing model generation...")
        test_prompt = "Plan a smooth path from (0,0) to (5,5) on the Euclidean plane."
        result = generate_text(model, tokenizer, test_prompt)
        print(f"\nPrompt: {test_prompt}")
        print(f"Generated: {result}")
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()

