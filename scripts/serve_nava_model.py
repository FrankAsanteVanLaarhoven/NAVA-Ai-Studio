# -*- coding: utf-8 -*-
"""
NAVA Model Serving Script
Serves the fine-tuned NAVA-Instruct model using vLLM or HuggingFace
"""

import argparse
import json
import os
from pathlib import Path
from typing import Optional

def serve_with_vllm(model_path, port=8080, host="0.0.0.0"):
    """Serve model with vLLM (fastest, recommended)"""
    from vllm import LLM, SamplingParams
    from transformers import AutoTokenizer
    import uvicorn
    from fastapi import FastAPI, Request
    from fastapi.responses import JSONResponse
    
    print(f"🚀 Loading model from {model_path}...")
    llm = LLM(
        model=model_path,
        tensor_parallel_size=1,
        gpu_memory_utilization=0.9,
        max_model_len=2048,
        trust_remote_code=True,
    )
    
    # Load tokenizer for chat template
    tokenizer = AutoTokenizer.from_pretrained(model_path, trust_remote_code=True)
    
    app = FastAPI(title="NAVA-Instruct Model Server")
    
    @app.post("/v1/completions")
    async def completions(request: Request):
        data = await request.json()
        prompt = data.get("prompt", "")
        temperature = data.get("temperature", 0.2)
        max_tokens = data.get("max_tokens", 512)
        
        sampling_params = SamplingParams(
            temperature=temperature,
            max_tokens=max_tokens,
            stop=["### Instruction:", "### Input:"],
        )
        
        outputs = llm.generate([prompt], sampling_params)
        generated_text = outputs[0].outputs[0].text
        
        return JSONResponse({
            "choices": [{
                "text": generated_text,
                "finish_reason": "stop"
            }]
        })
    
    @app.post("/v1/chat/completions")
    async def chat_completions(request: Request):
        data = await request.json()
        messages = data.get("messages", [])
        temperature = data.get("temperature", 0.2)
        max_tokens = data.get("max_tokens", 512)
        
        # Use tokenizer's chat template if available (for fine-tuned models)
        if hasattr(tokenizer, 'apply_chat_template'):
            try:
                formatted_prompt = tokenizer.apply_chat_template(
                    messages,
                    tokenize=False,
                    add_generation_prompt=True,
                )
            except Exception:
                # Fallback to manual formatting
                prompt = ""
                for msg in messages:
                    role = msg.get("role", "")
                    content = msg.get("content", "")
                    if role == "system":
                        prompt += f"### System:\n{content}\n\n"
                    elif role == "user":
                        prompt += f"### Instruction:\n{content}\n\n### Response:\n"
                    elif role == "assistant":
                        prompt += f"{content}\n\n"
                formatted_prompt = prompt
        else:
            # Manual formatting for models without chat template
            prompt = ""
            for msg in messages:
                role = msg.get("role", "")
                content = msg.get("content", "")
                if role == "system":
                    prompt += f"### System:\n{content}\n\n"
                elif role == "user":
                    prompt += f"### Instruction:\n{content}\n\n### Response:\n"
                elif role == "assistant":
                    prompt += f"{content}\n\n"
            formatted_prompt = prompt
        
        sampling_params = SamplingParams(
            temperature=temperature,
            max_tokens=max_tokens,
            stop=["### Instruction:", "### Input:", "<|end_of_text|>"],
        )
        
        outputs = llm.generate([formatted_prompt], sampling_params)
        generated_text = outputs[0].outputs[0].text
        
        # Extract only the generated part if needed
        if formatted_prompt in generated_text:
            generated_text = generated_text.split(formatted_prompt)[-1].strip()
        
        return JSONResponse({
            "choices": [{
                "message": {
                    "role": "assistant",
                    "content": generated_text
                },
                "finish_reason": "stop"
            }],
            "usage": {
                "prompt_tokens": len(prompt.split()),
                "completion_tokens": len(generated_text.split()),
                "total_tokens": len(prompt.split()) + len(generated_text.split())
            }
        })
    
    @app.get("/health")
    async def health():
        return {"status": "healthy", "model": model_path}
    
    print(f"✅ Model loaded! Starting server on {host}:{port}")
    uvicorn.run(app, host=host, port=port)

def serve_with_huggingface(model_path, port=8080, host="0.0.0.0"):
    """Serve model with HuggingFace Transformers (slower but more compatible)"""
    from transformers import AutoModelForCausalLM, AutoTokenizer
    import torch
    import uvicorn
    from fastapi import FastAPI, Request
    from fastapi.responses import JSONResponse
    
    print(f"🚀 Loading model from {model_path}...")
    tokenizer = AutoTokenizer.from_pretrained(
        model_path,
        trust_remote_code=True,
    )
    
    # Ensure pad token is set
    if tokenizer.pad_token is None:
        tokenizer.pad_token = tokenizer.eos_token
    tokenizer.padding_side = "right"
    
    # Determine dtype
    if torch.cuda.is_available():
        torch_dtype = torch.bfloat16
    else:
        torch_dtype = torch.float32
    
    model = AutoModelForCausalLM.from_pretrained(
        model_path,
        torch_dtype=torch_dtype,
        device_map="auto",
        trust_remote_code=True,
    )
    
    # Disable cache for inference
    model.config.use_cache = False
    
    app = FastAPI(title="NAVA-Instruct Model Server")
    
    @app.post("/v1/completions")
    async def completions(request: Request):
        data = await request.json()
        prompt = data.get("prompt", "")
        temperature = data.get("temperature", 0.2)
        max_tokens = data.get("max_tokens", 512)
        
        inputs = tokenizer(prompt, return_tensors="pt").to(model.device)
        outputs = model.generate(
            **inputs,
            max_new_tokens=max_tokens,
            temperature=temperature,
            do_sample=True,
            pad_token_id=tokenizer.eos_token_id,
        )
        generated_text = tokenizer.decode(outputs[0][inputs.input_ids.shape[1]:], skip_special_tokens=True)
        
        return JSONResponse({
            "choices": [{
                "text": generated_text,
                "finish_reason": "stop"
            }]
        })
    
    @app.post("/v1/chat/completions")
    async def chat_completions(request: Request):
        data = await request.json()
        messages = data.get("messages", [])
        temperature = data.get("temperature", 0.2)
        max_tokens = data.get("max_tokens", 512)
        
        # Use tokenizer's chat template if available (for fine-tuned models)
        if hasattr(tokenizer, 'apply_chat_template'):
            try:
                # Format messages using chat template
                formatted_prompt = tokenizer.apply_chat_template(
                    messages,
                    tokenize=False,
                    add_generation_prompt=True,
                )
            except Exception:
                # Fallback to manual formatting
                prompt = ""
                for msg in messages:
                    role = msg.get("role", "")
                    content = msg.get("content", "")
                    if role == "system":
                        prompt += f"### System:\n{content}\n\n"
                    elif role == "user":
                        prompt += f"### Instruction:\n{content}\n\n### Response:\n"
                    elif role == "assistant":
                        prompt += f"{content}\n\n"
                formatted_prompt = prompt
        else:
            # Manual formatting for models without chat template
            prompt = ""
            for msg in messages:
                role = msg.get("role", "")
                content = msg.get("content", "")
                if role == "system":
                    prompt += f"### System:\n{content}\n\n"
                elif role == "user":
                    prompt += f"### Instruction:\n{content}\n\n### Response:\n"
                elif role == "assistant":
                    prompt += f"{content}\n\n"
            formatted_prompt = prompt
        
        inputs = tokenizer(formatted_prompt, return_tensors="pt").to(model.device)
        
        # Generate with proper dtype handling
        with torch.no_grad():
            if torch.cuda.is_available():
                with torch.autocast(device_type="cuda", dtype=torch.bfloat16):
                    outputs = model.generate(
                        **inputs,
                        max_new_tokens=max_tokens,
                        temperature=temperature,
                        do_sample=True,
                        top_p=0.95,
                        pad_token_id=tokenizer.pad_token_id,
                    )
            else:
                outputs = model.generate(
                    **inputs,
                    max_new_tokens=max_tokens,
                    temperature=temperature,
                    do_sample=True,
                    top_p=0.95,
                    pad_token_id=tokenizer.pad_token_id,
                )
        
        generated_text = tokenizer.decode(outputs[0], skip_special_tokens=True)
        
        # Extract only the generated part (after the prompt)
        if formatted_prompt in generated_text:
            generated_text = generated_text.split(formatted_prompt)[-1].strip()
        
        return JSONResponse({
            "choices": [{
                "message": {
                    "role": "assistant",
                    "content": generated_text
                },
                "finish_reason": "stop"
            }],
            "usage": {
                "prompt_tokens": len(prompt.split()),
                "completion_tokens": len(generated_text.split()),
                "total_tokens": len(prompt.split()) + len(generated_text.split())
            }
        })
    
    @app.get("/health")
    async def health():
        return {"status": "healthy", "model": model_path}
    
    print(f"✅ Model loaded! Starting server on {host}:{port}")
    uvicorn.run(app, host=host, port=port)

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

def main():
    parser = argparse.ArgumentParser(description="Serve NAVA-Instruct model")
    parser.add_argument("--model", type=str, default=None, help="Path to fine-tuned model (auto-detects if not provided)")
    parser.add_argument("--port", type=int, default=8080, help="Server port")
    parser.add_argument("--host", type=str, default="0.0.0.0", help="Server host")
    parser.add_argument("--backend", type=str, choices=["vllm", "huggingface"], default="huggingface", help="Backend to use")
    
    args = parser.parse_args()
    
    # Auto-detect model path if not provided
    if args.model is None:
        model_path = find_model_path()
        if model_path is None:
            print("❌ Error: NAVA-Instruct model not found.")
            print("\n💡 Please specify --model path or ensure model is in one of:")
            print("   - ./models/nava-llama-3.1-8b-instruct-merged")
            print("   - ./models/nava-instruct-7b-merged")
            print("   - Set NAVA_MODEL_PATH environment variable")
            return
        args.model = model_path
        print(f"✅ Auto-detected model: {args.model}")
    
    if args.backend == "vllm":
        serve_with_vllm(args.model, args.port, args.host)
    else:
        serve_with_huggingface(args.model, args.port, args.host)

if __name__ == "__main__":
    main()

