# 🤗 HuggingFace Models - Complete Guide

## Available HuggingFace Models

### Currently Configured (in backend-initializer.ts):
1. **Mistral 7B Instruct** (`mistralai/Mistral-7B-Instruct-v0.2`)
   - Default HuggingFace model
   - Always available (no API key)
   - Good for general chat and code

### Available in free-llm-service.ts (but not in dropdown):
1. **Mistral 7B** (`mistralai/Mistral-7B-Instruct-v0.2`)
2. **Llama 7B** (`meta-llama/Llama-2-7b-chat-hf`)
3. **CodeLlama** (`codellama/CodeLlama-7b-Instruct-hf`)

## Why Only One Shows in Dropdown?

The model dropdown only shows models from `model-config-service.ts`.
Currently, HuggingFace models are NOT listed there - only added as a backend fallback.

## Solution: Add HuggingFace Models to Dropdown

We need to add HuggingFace models to the defaultModels list in model-config-service.ts.
