# 🚀 QUICK FIX - Stop Generic Responses

## The Problem
You're seeing generic error messages instead of real AI responses.

## Why It Happens
- Selected model (e.g., "Gemini 2.5 Pro") requires API key
- No API key → backend not created
- Backend lookup fails → shows error message
- Error message looks like "generic response"

## ✅ IMMEDIATE SOLUTION

### Use HuggingFace Models (Works Right Now!)

1. **Open AI Panel**
2. **Click model dropdown** (top of AI panel)
3. **Select any HuggingFace model:**
   - Mistral 7B (Free)
   - Llama 7B (Free)
   - CodeLlama (Free)
4. **Ask your question**
5. **Get REAL AI response!** ✅

---

## 🔑 To Use Better Models (Gemini, GPT, etc.)

### Step 1: Get API Keys
- **Gemini API Key** (FREE): https://ai.google.dev/
  - Use account: F.Van-Laarhoven2@newcastle.ac.uk
- **OpenRouter API Key**: https://openrouter.ai/
  - For Gemini 2.5 Pro, GPT-4o, Claude, etc.

### Step 2: Add Keys in IDE
1. Open AI Panel
2. Click Settings icon (⚙️)
3. Paste API keys
4. Save

### Step 3: Select Model
- **Gemini 1.5 Pro/Flash** → Use Gemini API key
- **Gemini 2.5 Pro** → Use OpenRouter API key
- **GPT-4o, Claude** → Use OpenRouter API key

---

## 🧪 Test It Works

### Test HuggingFace (Should Work Now)
```
1. Select HuggingFace model
2. Ask: "Hello, what can you help with?"
3. Expected: Real AI response (not error)
```

### Test with API Key
```
1. Add Gemini API key
2. Select Gemini 1.5 Pro
3. Ask question
4. Expected: Real AI response from Gemini
```

---

## 🔍 Debug in Browser Console

Open DevTools → Console, run:
```javascript
// Check available backends
backendRegistry.getAvailableBackends().map(b => b.id)

// Check API keys
modelConfigService.getProviderApiKey('gemini')
modelConfigService.getProviderApiKey('openrouter')
```

---

## ✅ Summary

**RIGHT NOW:** Use HuggingFace models (free, works immediately)
**LATER:** Add API keys for better models (Gemini, GPT, etc.)

**The generic responses were actually error messages.**
**Now with HuggingFace fallback, you'll get real AI responses!**
