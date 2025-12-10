# NAVA IDE - State of the Art Architecture

## Overview

This document describes the architecture of the NAVA-first IDE, designed to provide a "Unity for navigation calculus" experience with an embedded copilot.

## Core Principles

1. **NAVA-First**: All AI interactions default to NAVA/VΛNC code generation
2. **Model-Agnostic**: Unified interface for local and external models
3. **Live Preview**: Real-time visualization of NAVA programs
4. **Tight Loop**: Prompt → Code → Run → Visualize → Refine

## Architecture Layers

### 1. Frontend (IDE Shell)

**Tech Stack**: React + TypeScript + Vite

**Core Components**:
- **Monaco Editor**: Custom NAVA language support
- **AI Assistant Panel**: Chat interface with tool calling
- **Preview Panel**: 2D/3D canvas for visualization (TODO)
- **Scenario Panel**: Test multiple variants (TODO)
- **Learning Mode**: Educational explanations (TODO)

### 2. Backend Services

#### 2.1 NAVA Runtime Service (`nava-runtime-service.ts`)

Executes NAVA code and returns results for preview.

**Key Methods**:
- `runNAVA(code, options)`: Execute NAVA code, returns path, obstacles, cost grid
- `analyzeNAVA(code)`: Get invariants, costs, stability metrics

**Returns**:
```typescript
{
  success: boolean;
  path: PathPoint[];
  obstacles: Obstacle[];
  costGrid?: CostGrid;
  metadata: {
    manifold: string;
    pathLength: number;
    executionTime: number;
    energy?: number;
    jerkMax?: number;
  };
  visualization?: VisualizationData;
}
```

#### 2.2 NAVA Preview Engine (`nava-preview-engine.ts`)

Converts runtime results into visualization data.

**Key Methods**:
- `toPreviewData(result)`: Convert runtime result to preview format
- `generateScenarios(baseCode, numVariants)`: Create scenario variants
- `generateTimingProfile(path, options)`: Create DAAT/PDL/RT-shields timing data
- `generateVectorField(path, obstacles, bounds)`: Generate vector field visualization

#### 2.3 NAVA AI Tools (`nava-ai-tools.ts`)

Tool definitions for AI models to call.

**Available Tools**:
1. `generate_nava_from_prompt`: Convert natural language to NAVA code
2. `optimise_nava_expression`: Optimize for speed/readability/stability
3. `add_invariants`: Add safety checks and invariants
4. `explain_code_to_student`: Explain at GCSE/A-level/PhD level
5. `create_scenario_variants`: Generate multiple test scenarios
6. `run_nava_preview`: Execute code and get preview data
7. `add_daat_contract`: Add DAAT timing contracts
8. `add_pdl_tiers`: Add PDL tier annotations

**Tool Execution Flow**:
1. User sends prompt to AI
2. AI model decides to call a tool
3. Tool is executed (e.g., generates NAVA code)
4. Tool result is sent back to model
5. Model provides final response with tool results

### 3. LLM Hub Architecture

#### 3.1 Unified Backend Interface

All LLM providers implement `LLMBackend` interface:

```typescript
interface LLMBackend {
  id: string;
  displayName: string;
  capabilities: Capability[];
  chat(request: ChatRequest): Promise<ChatResponse>;
  isAvailable(): boolean;
}
```

#### 3.2 Backend Registry

Manages all backends and provides unified access:

- `backendRegistry.chat(prompt, context, backendId, tools)`: Single entry point
- Automatically wraps all requests with NAVA system prompt
- Handles tool calling
- Tracks usage and costs

#### 3.3 Available Backends

1. **NAVA Local** (`nava-local-backend.ts`): Local 7B model (default)
2. **OpenAI** (`openai-backend.ts`): GPT-4, GPT-4 Turbo
3. **OpenRouter** (`openrouter-backend.ts`): Access to multiple models
4. **Gemini** (`gemini-backend.ts`): Google Gemini models
5. **HuggingFace** (`huggingface-backend.ts`): HF Inference API

#### 3.4 NAVA System Prompt

All requests are automatically wrapped with NAVA context:

- Explains NAVA/VΛNC syntax
- Provides examples
- Sets default behavior to NAVA code generation
- Includes current file/code context

### 4. AI Panel Integration

The AI panel (`AIPanePanel.tsx`) uses the LLM Hub:

1. User sends prompt
2. Panel calls `backendRegistry.chat()` with tools
3. If model returns tool calls:
   - Execute tools
   - Insert generated code into editor
   - Update preview with visualization data
   - Send tool results back to model
4. Display final response

**Key Features**:
- Tool calling support
- Automatic code insertion
- Preview updates
- Cost tracking
- Multi-model support

## Data Flow

### Prompt → NAVA Code Flow

```
User: "Plan a path from [0,0] to [5,5] avoiding obstacle at [2,2]"
  ↓
AI Panel → backendRegistry.chat(prompt, context, tools)
  ↓
Model decides to call: generate_nava_from_prompt
  ↓
Tool executes → Generates NAVA code
  ↓
Code inserted into editor via nava:insert-code event
  ↓
Editor updates
  ↓
Preview panel listens for nava:preview-update event
  ↓
Preview panel calls navaRuntimeService.runNAVA(code)
  ↓
Preview panel visualizes path, obstacles, cost grid
```

### Live Preview Flow

```
User edits NAVA code in editor
  ↓
Editor emits nava:code-changed event
  ↓
Preview panel listens for event
  ↓
Preview panel calls navaRuntimeService.runNAVA(code, { preview: true })
  ↓
Runtime service executes code
  ↓
Preview engine converts result to visualization data
  ↓
Preview panel renders on canvas (2D/3D)
```

## Implementation Status

### ✅ Completed

1. **NAVA Runtime Service**: Execute NAVA code, return paths/obstacles
2. **NAVA Preview Engine**: Convert results to visualization data
3. **NAVA AI Tools**: 8 tools for AI models
4. **LLM Hub**: Unified backend interface with tool calling
5. **AI Panel**: Integrated with LLM Hub and tools
6. **Tool Execution**: Automatic tool calling and code insertion

### 🚧 In Progress / TODO

1. **Preview Panel Component**: 2D/3D canvas for visualization
   - Render paths, obstacles, cost grids
   - Interactive timeline for DAAT/PDL
   - Vector field visualization

2. **Scenario Panel**: Test multiple variants
   - UI for selecting scenarios
   - Side-by-side comparison
   - Batch execution

3. **Learning Mode**: Educational explanations
   - Line-by-line code explanations
   - Figure annotations
   - Q&A generation

4. **DAAT/PDL/RT-shields Visualization**:
   - Timing overlays on paths
   - Tier switches visualization
   - Contract status indicators
   - Timeline scrubber

5. **Monaco Editor Enhancements**:
   - Semantic code intelligence (AST)
   - Go-to definition
   - Symbol search
   - Refactorings

6. **Local NAVA 7B Model**:
   - Fine-tune on NAVA docs/code
   - Serve with vLLM/llama.cpp
   - Integrate as default backend

## File Structure

```
src/
├── services/
│   ├── nava-runtime-service.ts      # Execute NAVA code
│   ├── nava-preview-engine.ts        # Convert to visualization data
│   ├── nava-ai-tools.ts              # AI tool definitions
│   └── llm-hub/
│       ├── types.ts                  # Unified interfaces
│       ├── backend-registry.ts      # Backend management
│       ├── nava-system-prompt.ts    # NAVA context wrapper
│       └── backends/
│           ├── nava-local-backend.ts
│           ├── openai-backend.ts
│           ├── openrouter-backend.ts
│           ├── gemini-backend.ts
│           └── huggingface-backend.ts
├── components/
│   ├── AI/
│   │   └── AIPanePanel.tsx          # AI chat with tool calling
│   └── Preview/                      # TODO: Preview panel
│       └── NAVAPreviewPanel.tsx
└── apps/
    └── univarm-starter/
        └── engine/
            └── nava-engine.ts        # NAVA path computation
```

## Next Steps

1. **Create Preview Panel Component**:
   - Use Three.js or Canvas API
   - Render paths, obstacles, cost grids
   - Add interactive controls

2. **Add Scenario Panel**:
   - UI for scenario selection
   - Batch execution
   - Comparison view

3. **Implement Learning Mode**:
   - Explanation generation
   - Figure annotations
   - Q&A system

4. **Add DAAT/PDL Visualization**:
   - Timing overlays
   - Tier switches
   - Contract status

5. **Enhance Monaco Editor**:
   - AST-based features
   - Semantic intelligence
   - Refactorings

6. **Fine-tune Local Model**:
   - Collect NAVA training data
   - Fine-tune 7B model
   - Deploy with vLLM

## Key Design Decisions

1. **Model-Agnostic**: All models go through LLM Hub, ensuring consistent behavior
2. **NAVA-First**: System prompt ensures all models default to NAVA code generation
3. **Tool-Based**: AI uses tools to interact with IDE, not direct code generation
4. **Event-Driven**: Components communicate via custom events
5. **Service-Oriented**: Clear separation between runtime, preview, and AI services

## Testing Strategy

1. **Unit Tests**: Test each service independently
2. **Integration Tests**: Test tool calling flow
3. **E2E Tests**: Test full prompt → code → preview flow
4. **Visual Regression**: Test preview rendering

## Performance Considerations

1. **Lazy Loading**: Load preview engine only when needed
2. **Debouncing**: Debounce code changes before preview update
3. **Caching**: Cache runtime results for unchanged code
4. **Streaming**: Stream AI responses for better UX

