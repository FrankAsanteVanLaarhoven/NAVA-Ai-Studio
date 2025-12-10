# AI Panel - Complete Architecture & Component List

## 📋 Table of Contents

1. [Overview](#overview)
2. [Architecture Diagram](#architecture-diagram)
3. [Component List](#component-list)
4. [LLM Hub Architecture](#llm-hub-architecture)
5. [Tool System](#tool-system)
6. [Backend Providers](#backend-providers)
7. [Services](#services)
8. [UI Components](#ui-components)
9. [Data Flow](#data-flow)
10. [Integration Points](#integration-points)

---

## Overview

The NAVA Studio IDE AI Panel is a **comprehensive, NAVA-first copilot** that provides intelligent code assistance, execution, visualization, and educational support. It uses a model-agnostic architecture that supports multiple LLM providers through a unified interface.

### Key Features

- ✅ **8 Specialized Tools** for IDE interaction
- ✅ **Multi-Model Support** (local + external providers)
- ✅ **NAVA-First Behavior** (defaults to NAVA code generation)
- ✅ **Educational Support** (GCSE to PhD levels)
- ✅ **Live Preview** (execute and visualize)
- ✅ **Scenario Management** (variants and testing)
- ✅ **Cost Tracking** (token usage and costs)
- ✅ **Free Mode** (works without API keys)

---

## Architecture Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                        NAVA Studio IDE                           │
│                                                                   │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │                    AI Panel UI                           │   │
│  │  (AIPanePanel.tsx, WelcomeScreen.tsx)                   │   │
│  └────────────────────┬─────────────────────────────────────┘   │
│                       │                                           │
│                       ▼                                           │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │                  LLM Hub (Unified)                       │   │
│  │  • Backend Registry                                       │   │
│  │  • NAVA System Prompt Wrapper                            │   │
│  │  • Tool Calling Handler                                   │   │
│  │  • Cost Tracker                                           │   │
│  └───────┬───────────────────────────────────────────────────┘   │
│          │                                                        │
│          ├──────────────────────────────────────────────┐        │
│          │                                              │        │
│          ▼                                              ▼        │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐            │
│  │ NAVA Local   │  │  OpenRouter  │  │   Gemini    │  ...       │
│  │ (Fine-tuned) │  │  (100+ models)│  │  (Google)   │            │
│  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘            │
│         │                 │                  │                    │
│         └─────────────────┴──────────────────┘                    │
│                           │                                        │
│                           ▼                                        │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │                  Tool Execution Layer                     │   │
│  │  • generate_nava_from_prompt                             │   │
│  │  • explain_nava_code                                     │   │
│  │  • run_nava_program                                      │   │
│  │  • optimise_nava_expression                              │   │
│  │  • generate_scenarios_from_code                         │   │
│  │  • fetch_nava_docs                                       │   │
│  │  • add_daat_contract                                     │   │
│  │  • add_pdl_tiers                                         │   │
│  └───────┬───────────────────────────────────────────────────┘   │
│          │                                                        │
│          ├──────────────────────────────────────────────┐        │
│          │                                              │        │
│          ▼                                              ▼        │
│  ┌──────────────┐                            ┌──────────────┐   │
│  │ NAVA Runtime │                            │  Preview     │   │
│  │   Service    │                            │   Engine     │   │
│  └──────────────┘                            └──────────────┘   │
│                                                                   │
└─────────────────────────────────────────────────────────────────┘
```

---

## Component List

### 1. UI Components

#### `src/components/AI/AIPanePanel.tsx`
**Main AI Panel Component**

**Features:**
- Chat interface with message history
- Model selection dropdown
- File attachment support
- Voice input/output
- Code insertion into editor
- Settings panel
- Cost tracking display
- Tool execution UI

**State Management:**
- `messages`: Conversation history
- `availableModels`: List of available models
- `selectedModel`: Currently selected model
- `isThinking`: Loading state
- `uploadedFiles`: Attached files

#### `src/components/AI/WelcomeScreen.tsx`
**Welcome/Onboarding Screen**

**Features:**
- Quick start examples
- Feature highlights
- Getting started guide

#### `src/components/AI/AIPanePanel.css`
**Styling for AI Panel**

---

### 2. LLM Hub Core

#### `src/services/llm-hub/types.ts`
**Type Definitions**

**Interfaces:**
- `LLMBackend`: Unified backend interface
- `ChatRequest`: Request format
- `ChatResponse`: Response format
- `ToolDefinition`: Tool schema
- `ToolCall`: Tool call structure
- `BackendConfig`: Backend configuration
- `Capability`: Backend capabilities ('code', 'maths', 'navcalc', 'chat', 'reasoning', 'multimodal')

#### `src/services/llm-hub/backend-registry.ts`
**Backend Registry (Central Hub)**

**Responsibilities:**
- Register and manage all backends
- Route requests to appropriate backend
- Wrap all requests with NAVA system prompt
- Handle tool calling
- Provide unified interface

**Key Methods:**
- `register(backend)`: Register a backend
- `getBackend(id)`: Get backend by ID
- `getDefaultBackend()`: Get default backend
- `chat(prompt, context, backendId, tools)`: Unified chat interface
- `streamChat()`: Streaming chat support

#### `src/services/llm-hub/nava-system-prompt.ts`
**NAVA System Prompt Wrapper**

**Contents:**
- `navaSystemPrompt`: Core system prompt
- `fewShotExamples`: Example conversations
- `buildNAVAMessages()`: Build messages with context
- `buildNAVAMessagesWithExamples()`: Build with few-shot examples

#### `src/services/llm-hub/cost-tracker.ts`
**Cost Tracking Service**

**Features:**
- Track token usage per provider
- Calculate costs based on provider pricing
- Display cost estimates in UI

#### `src/services/llm-hub/index.ts`
**LLM Hub Exports**

**Exports:**
- `backendRegistry`: Main registry instance
- `costTracker`: Cost tracking service
- All backend classes
- Type definitions

---

### 3. Backend Implementations

#### `src/services/llm-hub/backends/nava-local-backend.ts`
**NAVA Local Backend (Fine-tuned Model)**

**Configuration:**
- Base URL: `http://localhost:8080` (configurable)
- Model: `nava-instruct-7b`
- Capabilities: `['code', 'navcalc', 'chat', 'reasoning']`

**Features:**
- Connects to local model server
- Uses chat template for fine-tuned models
- Proper message formatting

#### `src/services/llm-hub/backends/openai-backend.ts`
**OpenAI Backend**

**Models Supported:**
- GPT-4, GPT-4 Turbo
- GPT-3.5

#### `src/services/llm-hub/backends/openrouter-backend.ts`
**OpenRouter Backend**

**Models Supported:**
- 100+ models via OpenRouter
- Claude 3.5 Sonnet
- GPT-4, GPT-5
- Llama 3 70B
- And many more

#### `src/services/llm-hub/backends/gemini-backend.ts`
**Google Gemini Backend**

**Models Supported:**
- Gemini 2.0 Flash
- Gemini 1.5 Pro
- Gemini 1.5 Flash

#### `src/services/llm-hub/backends/huggingface-backend.ts`
**HuggingFace Backend**

**Models Supported:**
- Mistral 7B
- Llama 2 7B
- CodeLlama
- Free models via Inference API

---

### 4. Tool System

#### `src/services/nava-ai-tools.ts`
**NAVA AI Tools Definitions**

**8 Available Tools:**

1. **`generate_nava_from_prompt`**
   - Convert natural language to NAVA code
   - Parameters: `prompt`, `context_code?`, `target_level`
   - Returns: Generated NAVA code

2. **`explain_nava_code`**
   - Explain code at different levels
   - Parameters: `code`, `audience` (gcse/a-level/undergrad/phd), `focus_question?`
   - Returns: Explanation text

3. **`run_nava_program`**
   - Execute NAVA code
   - Parameters: `code`, `max_steps?`, `sampling_dt?`
   - Returns: Path data, obstacles, metrics

4. **`optimise_nava_expression`**
   - Refactor/optimize code
   - Parameters: `code`, `goal` (readability/performance/stability/idiomatic_nava)
   - Returns: Optimized code

5. **`generate_scenarios_from_code`**
   - Create scenario variants
   - Parameters: `code`, `num_variants`, `variation_type`
   - Returns: Array of variant code

6. **`fetch_nava_docs`**
   - Search documentation
   - Parameters: `query`
   - Returns: Documentation snippets

7. **`add_daat_contract`**
   - Add DAAT timing contracts
   - Parameters: `code`, `frequency_hz`, `dmr_threshold?`
   - Returns: Code with DAAT contracts

8. **`add_pdl_tiers`**
   - Add PDL tier annotations
   - Parameters: `code`
   - Returns: Code with PDL tiers

#### `src/services/nava-tools-service.ts`
**NAVA Tools Service (Tool Execution)**

**Responsibilities:**
- Execute tool calls from AI
- Interface with NAVA runtime
- Return tool results to AI

---

### 5. Supporting Services

#### `src/services/nava-runtime-service.ts`
**NAVA Runtime Service**

**Methods:**
- `runNAVA(code)`: Execute NAVA code
- `analyzeNAVA(code)`: Analyze code for invariants

#### `src/services/nava-preview-engine.ts`
**NAVA Preview Engine**

**Features:**
- Convert runtime results to visualization data
- Generate scenario variants
- Create timing profiles

#### `src/services/nava-assistant-service.ts`
**NAVA Assistant Service**

**Features:**
- High-level NAVA code generation
- Code explanation
- Code optimization

#### `src/services/nava-agents-service.ts`
**NAVA Agents Service**

**Features:**
- Multi-agent orchestration
- Specialized agents for different tasks

#### `src/services/model-config-service.ts`
**Model Configuration Service**

**Features:**
- Manage model configurations
- API key storage
- Default model selection
- Custom model support

#### `src/services/free-llm-service.ts`
**Free LLM Service**

**Features:**
- Fallback to free models (HuggingFace)
- Works without API keys
- Basic code generation

#### `src/services/ai-code-generator-service.ts`
**AI Code Generator Service**

**Features:**
- Code generation with context
- Multi-file support
- Language detection

#### `src/services/voice-service.ts`
**Voice Service**

**Features:**
- Voice input (speech-to-text)
- Voice output (text-to-speech)

---

### 6. Model Server (Python)

#### `scripts/serve_nava_model.py`
**Model Serving Script**

**Backends:**
- HuggingFace (default)
- vLLM (faster, GPU required)

**Endpoints:**
- `POST /v1/chat/completions`: Chat completion
- `POST /v1/completions`: Legacy completion
- `GET /health`: Health check

#### `scripts/load_nava_model.py`
**Model Loader Utility**

**Features:**
- Auto-detect model path
- Load model and tokenizer
- Test generation

#### `scripts/start_nava_model.sh`
**Startup Script**

**Features:**
- Auto-detect model
- Port conflict detection
- Environment variable support

---

## LLM Hub Architecture

### Unified Interface

All backends implement the `LLMBackend` interface:

```typescript
interface LLMBackend {
  id: string;                    // Unique identifier
  displayName: string;            // Display name
  capabilities: Capability[];     // Supported capabilities
  chat(request: ChatRequest): Promise<ChatResponse>;
  streamChat?(request: ChatRequest): AsyncIterable<ChatChunk>;
  isAvailable(): boolean;
  getConfig(): BackendConfig;
}
```

### Request Flow

1. **User sends prompt** → AI Panel
2. **AI Panel** → Backend Registry
3. **Backend Registry** → Wraps with NAVA system prompt
4. **Backend Registry** → Routes to selected backend
5. **Backend** → Calls LLM API
6. **LLM** → Returns response (possibly with tool calls)
7. **Backend Registry** → Executes tools if needed
8. **Tool Results** → Sent back to LLM
9. **Final Response** → Returned to AI Panel
10. **AI Panel** → Displays response

### System Prompt Wrapper

All requests are automatically wrapped with:
- NAVA system prompt (defines NAVA-first behavior)
- Current file context (if available)
- Current code context (if available)
- Few-shot examples (optional)

---

## Tool System

### Tool Definition Format

```typescript
{
  type: 'function',
  function: {
    name: 'tool_name',
    description: 'Tool description',
    parameters: {
      type: 'object',
      properties: {
        param1: { type: 'string', description: '...' },
        param2: { type: 'number', description: '...' }
      },
      required: ['param1']
    }
  }
}
```

### Tool Execution Flow

1. **AI decides to call tool** → Returns `toolCalls` in response
2. **Backend Registry** → Extracts tool calls
3. **Tool Service** → Executes tool with parameters
4. **Tool Result** → Formatted as tool message
5. **Tool Message** → Sent back to LLM
6. **LLM** → Generates final response using tool results

### Tool Categories

**Code Generation:**
- `generate_nava_from_prompt`
- `generate_scenarios_from_code`

**Code Understanding:**
- `explain_nava_code`
- `fetch_nava_docs`

**Code Execution:**
- `run_nava_program`

**Code Improvement:**
- `optimise_nava_expression`

**Advanced Features:**
- `add_daat_contract`
- `add_pdl_tiers`

---

## Backend Providers

### 1. NAVA Local (Default)

**Provider:** `nava-local`  
**Model:** `nava-instruct-7b`  
**Base URL:** `http://localhost:8080`  
**Capabilities:** `['code', 'navcalc', 'chat', 'reasoning']`

**Status:**
- ✅ Fine-tuned on NAVA-Instruct dataset
- ✅ Uses Llama chat template
- ✅ Optimized for NAVA code generation
- ✅ Runs locally (no API costs)

### 2. OpenRouter

**Provider:** `openrouter`  
**Models:** 100+ models  
**Capabilities:** `['code', 'maths', 'chat', 'reasoning']`

**Popular Models:**
- `anthropic/claude-3.5-sonnet` (Best for coding)
- `openai/gpt-4o`
- `openai/gpt-5-codex`
- `meta-llama/llama-3-70b-instruct`

### 3. OpenAI

**Provider:** `openai`  
**Models:** GPT-4, GPT-4 Turbo, GPT-3.5  
**Capabilities:** `['code', 'maths', 'chat', 'reasoning']`

### 4. Google Gemini

**Provider:** `gemini`  
**Models:** Gemini 2.0, 1.5 Pro, 1.5 Flash  
**Capabilities:** `['code', 'maths', 'chat', 'multimodal']`

**Features:**
- Free tier available
- 1M context window
- Multimodal support

### 5. HuggingFace

**Provider:** `huggingface`  
**Models:** Mistral, Llama, CodeLlama  
**Capabilities:** `['code', 'maths', 'chat']`

**Features:**
- Free for public models
- No API key required
- Inference API

---

## Services

### Core Services

1. **`backendRegistry`** (`llm-hub/backend-registry.ts`)
   - Central hub for all LLM backends
   - Routes requests to appropriate backend
   - Handles tool calling

2. **`costTracker`** (`llm-hub/cost-tracker.ts`)
   - Tracks token usage
   - Calculates costs
   - Provider-specific pricing

3. **`modelConfigService`** (`model-config-service.ts`)
   - Manages model configurations
   - API key storage
   - Default model selection

4. **`navaToolsService`** (`nava-tools-service.ts`)
   - Executes AI tool calls
   - Interfaces with NAVA runtime

5. **`navaRuntimeService`** (`nava-runtime-service.ts`)
   - Executes NAVA code
   - Returns path data and metrics

6. **`navaPreviewEngine`** (`nava-preview-engine.ts`)
   - Converts runtime data to visualization
   - Generates scenario variants

7. **`navaAssistantService`** (`nava-assistant-service.ts`)
   - High-level NAVA assistance
   - Code generation and explanation

8. **`freeLLMService`** (`free-llm-service.ts`)
   - Fallback to free models
   - Works without API keys

9. **`voiceService`** (`voice-service.ts`)
   - Voice input/output
   - Speech-to-text, text-to-speech

---

## UI Components

### AIPanePanel.tsx

**Main Features:**
- Chat interface
- Model selection
- File attachments
- Voice controls
- Settings panel
- Cost display
- Tool execution UI

**State:**
```typescript
- messages: Message[]
- availableModels: AIModelWithProvider[]
- selectedModel: string
- isThinking: boolean
- uploadedFiles: UploadedFile[]
- isVoiceEnabled: boolean
- isListening: boolean
```

**Key Methods:**
- `handleSend()`: Send message
- `handleCodeGeneration()`: Generate code
- `handleFileUpload()`: Upload files
- `handleVoiceToggle()`: Toggle voice
- `handleModelChange()`: Change model

### WelcomeScreen.tsx

**Features:**
- Quick start examples
- Feature highlights
- Getting started guide

---

## Data Flow

### 1. User Input → AI Response

```
User Types Message
    ↓
AIPanePanel.handleSend()
    ↓
backendRegistry.chat(prompt, context, backendId, tools)
    ↓
buildNAVAMessages() [Adds system prompt + context]
    ↓
Backend.chat(request) [Calls LLM API]
    ↓
LLM Returns Response [May include tool calls]
    ↓
If tool calls → Execute tools → Send results back
    ↓
Final Response → AIPanePanel
    ↓
Display in Chat UI
```

### 2. Tool Execution Flow

```
LLM Returns Tool Call
    ↓
Extract tool name and parameters
    ↓
navaToolsService.executeTool(toolName, params)
    ↓
Tool Implementation (e.g., run_nava_program)
    ↓
navaRuntimeService.runNAVA(code)
    ↓
Return Tool Result
    ↓
Format as Tool Message
    ↓
Send back to LLM
    ↓
LLM Generates Final Response
```

### 3. Code Generation Flow

```
User: "Plan a path from (0,0) to (5,5)"
    ↓
AI decides to call generate_nava_from_prompt
    ↓
Tool executed → Returns NAVA code
    ↓
AI formats response with code
    ↓
AIPanePanel detects code block
    ↓
Optionally calls run_nava_program
    ↓
Updates preview panel
    ↓
Inserts code into editor
```

---

## Integration Points

### 1. Editor Integration

**Event:** `nava:insert-code`
```typescript
window.dispatchEvent(new CustomEvent('nava:insert-code', {
  detail: { code, language, explanation }
}));
```

### 2. Preview Panel Integration

**Service:** `navaPreviewEngine`
- Receives runtime data
- Converts to visualization format
- Updates preview panel

### 3. Model Server Integration

**Endpoint:** `http://localhost:8080/v1/chat/completions`

**Request Format:**
```json
{
  "messages": [
    {"role": "system", "content": "..."},
    {"role": "user", "content": "..."}
  ],
  "temperature": 0.7,
  "max_tokens": 512
}
```

### 4. Settings Integration

**Storage:** `localStorage`
- Model preferences
- API keys (encrypted)
- Default backend
- Tool preferences

---

## File Structure

```
src/
├── components/
│   └── AI/
│       ├── AIPanePanel.tsx      # Main AI panel
│       ├── AIPanePanel.css      # Styles
│       ├── WelcomeScreen.tsx    # Welcome screen
│       └── WelcomeScreen.css    # Welcome styles
│
├── services/
│   ├── llm-hub/
│   │   ├── types.ts              # Type definitions
│   │   ├── backend-registry.ts   # Central hub
│   │   ├── nava-system-prompt.ts  # System prompt
│   │   ├── cost-tracker.ts       # Cost tracking
│   │   ├── index.ts              # Exports
│   │   └── backends/
│   │       ├── nava-local-backend.ts
│   │       ├── openai-backend.ts
│   │       ├── openrouter-backend.ts
│   │       ├── gemini-backend.ts
│   │       └── huggingface-backend.ts
│   │
│   ├── nava-ai-tools.ts          # Tool definitions
│   ├── nava-tools-service.ts    # Tool execution
│   ├── nava-runtime-service.ts  # NAVA execution
│   ├── nava-preview-engine.ts   # Preview engine
│   ├── nava-assistant-service.ts
│   ├── nava-agents-service.ts
│   ├── model-config-service.ts
│   ├── free-llm-service.ts
│   ├── ai-code-generator-service.ts
│   └── voice-service.ts
│
scripts/
├── serve_nava_model.py          # Model server
├── load_nava_model.py           # Model loader
└── start_nava_model.sh          # Startup script
```

---

## Summary

The AI Panel is a **comprehensive, production-ready NAVA copilot** with:

✅ **8 Specialized Tools** for IDE interaction  
✅ **5 Backend Providers** (NAVA Local, OpenRouter, OpenAI, Gemini, HuggingFace)  
✅ **100+ Available Models** via OpenRouter  
✅ **Unified Architecture** through LLM Hub  
✅ **NAVA-First Behavior** via system prompt  
✅ **Tool Calling System** for IDE integration  
✅ **Cost Tracking** for all providers  
✅ **Free Mode** (works without API keys)  
✅ **Voice Support** (input/output)  
✅ **File Attachments**  
✅ **Live Preview** integration  
✅ **Educational Support** (GCSE to PhD)  

This architecture makes it a **state-of-the-art, NAVA-specialized copilot** that rivals Cursor's AI but is optimized for navigation calculus.

