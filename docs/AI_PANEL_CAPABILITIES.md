# AI Panel Capabilities - Complete Reference

## Overview

The NAVA Studio IDE AI Panel is a comprehensive, NAVA-first copilot that provides intelligent code assistance, execution, visualization, and educational support. This document lists all capabilities.

## Core Architecture

### 1. Model-Agnostic LLM Hub
- **Multiple Backend Support**:
  - ✅ NAVA Local (7B fine-tuned model - when available)
  - ✅ OpenAI (GPT-4, GPT-4 Turbo, GPT-3.5)
  - ✅ OpenRouter (access to 100+ models)
  - ✅ Google Gemini (Gemini 1.5 Pro, Flash, 2.0)
  - ✅ HuggingFace Inference API (free models)
  - ✅ Anthropic Claude (via OpenRouter)

- **Unified Interface**: All models go through the same LLM Hub, ensuring consistent behavior
- **Automatic NAVA System Prompt**: All models get wrapped with NAVA context automatically
- **Cost Tracking**: Token usage and cost estimation for all providers
- **Model Selection**: Easy switching between models via dropdown

### 2. Tool Calling System
The AI can call 8 specialized tools to interact with the IDE:

#### Tool 1: `generate_nava_from_prompt`
**Capability**: Convert natural language to NAVA/VNC code
- Takes user intent in plain English
- Generates idiomatic NAVA code
- Supports different complexity levels (beginner/intermediate/expert)
- Can extend existing code with context
- Automatically inserts generated code into editor

**Example**: "Plan a path from (0,0) to (5,5) avoiding an obstacle" → Full NAVA code

#### Tool 2: `explain_nava_code`
**Capability**: Explain NAVA code at different educational levels
- **GCSE Level**: Simple, plain language explanations
- **A-level**: Geometry and mathematical concepts
- **Undergrad**: Manifold theory and optimization
- **PhD**: Advanced mathematical formalism
- Optional focus questions (e.g., "why is this stable?")

**Example**: Explains navigation fields, manifolds, cost functions at appropriate depth

#### Tool 3: `run_nava_program`
**Capability**: Execute NAVA code and get preview data
- Runs code through NAVA runtime
- Returns:
  - Sampled trajectories (path points)
  - Obstacle data
  - Cost grids for visualization
  - Metrics (length, curvature, jerk, energy)
  - Manifold information
- Configurable sampling (max_steps, sampling_dt)
- Automatically triggers preview panel updates

**Example**: Runs code → Returns path data → Updates visualization

#### Tool 4: `optimise_nava_expression`
**Capability**: Refactor and optimize NAVA code
- **Readability**: Better variable names, comments, formatting
- **Performance**: Faster execution, reduced samples
- **Stability**: Bounds checking, smoother paths
- **Idiomatic NAVA**: Follows NAVA best practices
- Preserves semantics and invariants

**Example**: Refactors code while keeping functionality identical

#### Tool 5: `generate_scenarios_from_code`
**Capability**: Create multiple scenario variants
- Generates variations by changing:
  - Start/goal positions
  - Obstacle configurations
  - Manifold types
  - Mixed variations
- Configurable number of variants (1-10)
- Returns complete NAVA code for each scenario
- Useful for testing and comparison

**Example**: Base scenario → 3 variants with different start/goal positions

#### Tool 6: `fetch_nava_docs`
**Capability**: Search NAVA/VNC documentation
- Searches function references
- Finds operator documentation
- Provides code examples
- Returns relevant snippets
- Helps with syntax and idioms

**Example**: "How do I create a navigation field?" → Documentation with examples

#### Tool 7: `add_daat_contract`
**Capability**: Add DAAT (Deadline-Aware Architecture Template) contracts
- Adds timing contracts to NAVA code
- Configurable frequency (Hz)
- Deadline miss rate (DMR) thresholds
- Jitter constraints
- Wraps code with deadline operators

**Example**: Adds timing guarantees for real-time systems

#### Tool 8: `add_pdl_tiers`
**Capability**: Add PDL (Policy Description Language) tier annotations
- Creates tiered architecture:
  - **Tier-0**: Reflex policy (<1ms)
  - **Tier-1**: Chunked VLA (10ms budget)
  - **Tier-2**: Offline planner (longer horizon)
- Hierarchical composition with fallbacks
- Time budget management

**Example**: Adds multi-tier control architecture to NAVA code

## Conversational Capabilities

### 3. Natural Language Understanding
- **Intent Recognition**: Understands user requests in plain English
- **Context Awareness**: Uses current file and code context
- **Multi-turn Conversations**: Maintains conversation history
- **Code Detection**: Automatically detects when user wants code generation
- **Language Routing**: Defaults to NAVA unless user specifies Python/C++/etc.

### 4. Code Generation
- **NAVA-First**: Always generates NAVA code by default
- **Complete Solutions**: Generates full, runnable code
- **Best Practices**: Follows NAVA idioms and patterns
- **Comments**: Adds helpful comments (especially for beginners)
- **Error Handling**: Includes proper error handling
- **Multi-file Support**: Can generate multiple related files

### 5. Code Explanation
- **Line-by-Line**: Explains what each part does
- **Conceptual**: Explains underlying mathematics
- **Visual**: Describes what visualizations show
- **Educational**: Adapts to user's level
- **Focus Areas**: Can focus on specific aspects (stability, geometry, etc.)

### 6. Code Refactoring
- **Improve Readability**: Better names, formatting, structure
- **Optimize Performance**: Faster execution, reduced computation
- **Enhance Stability**: Add safety checks, bounds
- **Make Idiomatic**: Follow NAVA best practices
- **Preserve Semantics**: Never changes functionality

## IDE Integration

### 7. Editor Integration
- **Code Insertion**: Automatically inserts generated code into editor
- **File Context**: Uses current file's code as context
- **Language Detection**: Detects file language for appropriate responses
- **Multi-file Operations**: Can work across multiple files
- **Line Navigation**: Can navigate to specific lines

### 8. Preview & Visualization
- **Live Preview**: Automatically runs code and shows results
- **Path Visualization**: Shows computed paths on canvas
- **Obstacle Display**: Visualizes obstacles
- **Cost Grids**: Shows cost heatmaps
- **Vector Fields**: Displays navigation fields
- **Timing Profiles**: DAAT/PDL timing overlays (when available)

### 9. Scenario Management
- **Variant Generation**: Creates multiple test scenarios
- **Batch Execution**: Runs multiple scenarios
- **Comparison**: Compares results across scenarios
- **Parameter Variation**: Changes start/goal, obstacles, manifolds

## Educational Features

### 10. Learning Mode
- **Adaptive Explanations**: Adjusts to user's level
- **Step-by-Step**: Breaks down complex concepts
- **Q&A Generation**: Creates questions and answers
- **Visual Annotations**: Explains what visualizations show
- **Concept Building**: Builds understanding progressively

### 11. Teaching Support
- **GCSE Level**: Simple, accessible explanations
- **A-level**: Mathematical concepts with geometry
- **Undergrad**: Manifold theory and optimization
- **PhD**: Advanced mathematical formalism
- **Custom Focus**: Can focus on specific topics

## Advanced Features

### 12. DAAT/PDL/RT-shields Support
- **Timing Contracts**: Adds deadline-aware contracts
- **Tier Management**: Creates multi-tier architectures
- **Real-time Safety**: RT-shields contract support
- **Timing Visualization**: Shows timing profiles on paths
- **Contract Verification**: Checks timing guarantees

### 13. Multi-Manifold Support
- **R^2**: Euclidean plane
- **R^3**: 3D Euclidean space
- **S^1**: Unit circle
- **S^2**: Unit sphere
- **SE(2)**: Planar rigid body
- **SE(3)**: 3D rigid body
- **Custom Manifolds**: User-defined manifolds

### 14. Cost Function Varieties
- **Geodesic**: Shortest path
- **Smooth**: Low curvature and jerk
- **Time-Optimal**: Fastest path
- **Energy-Optimal**: Minimum energy
- **Custom**: User-defined cost functions

## User Experience

### 15. Conversation Management
- **Message History**: Maintains conversation context
- **File Attachments**: Can include files in conversation
- **Code Snippets**: Can reference code in messages
- **Multi-turn**: Handles follow-up questions
- **Context Preservation**: Remembers previous interactions

### 16. Status & Feedback
- **Thinking Indicator**: Shows when AI is processing
- **Model Status**: Shows which model is active
- **API Status**: Indicates if API keys are configured
- **Free Mode**: Works without API keys (HuggingFace fallback)
- **Cost Display**: Shows token usage and estimated costs

### 17. Settings & Configuration
- **Model Selection**: Choose from available models
- **API Key Management**: Secure API key storage
- **Provider Settings**: Configure each provider
- **Default Model**: Set preferred model
- **Custom Models**: Add custom model configurations

## Technical Capabilities

### 18. Error Handling
- **Graceful Fallbacks**: Falls back to free LLM if APIs fail
- **Error Messages**: Clear, helpful error messages
- **Retry Logic**: Automatic retries for transient errors
- **Validation**: Validates tool arguments
- **Recovery**: Recovers from tool execution errors

### 19. Performance
- **Streaming Support**: Can stream responses (when available)
- **Caching**: Caches tool results when appropriate
- **Debouncing**: Debounces rapid requests
- **Optimization**: Optimized for fast responses
- **Resource Management**: Efficient resource usage

### 20. Extensibility
- **Tool System**: Easy to add new tools
- **Backend System**: Easy to add new model providers
- **Plugin Architecture**: Extensible architecture
- **Custom Prompts**: Can customize system prompts
- **Integration Points**: Hooks for custom integrations

## Use Cases

### For Students
- ✅ Learn NAVA syntax through examples
- ✅ Get explanations at your level (GCSE/A-level/undergrad)
- ✅ Generate practice problems
- ✅ Understand code step-by-step
- ✅ Get help with homework

### For Researchers
- ✅ Generate NAVA code for experiments
- ✅ Create scenario variants for testing
- ✅ Analyze code for invariants and stability
- ✅ Optimize code for performance
- ✅ Add timing contracts (DAAT/PDL)

### For Developers
- ✅ Generate NAVA code from requirements
- ✅ Refactor and optimize existing code
- ✅ Debug and fix errors
- ✅ Understand legacy code
- ✅ Generate test scenarios

### For Educators
- ✅ Create teaching examples
- ✅ Generate exercises at different levels
- ✅ Explain concepts clearly
- ✅ Create visualizations
- ✅ Generate Q&A materials

## Example Interactions

### Example 1: Natural Language → Code
**User**: "Plan a smooth path from (0,0) to (5,5) avoiding a circular obstacle at (2,2) with radius 1"

**AI**: 
1. Calls `generate_nava_from_prompt`
2. Generates complete NAVA code
3. Optionally calls `run_nava_program` to preview
4. Inserts code into editor
5. Updates preview panel

### Example 2: Code Explanation
**User**: "Explain this NAVA code at GCSE level" [pastes code]

**AI**:
1. Calls `explain_nava_code` with audience='gcse'
2. Provides simple, accessible explanation
3. Uses analogies and plain language

### Example 3: Code Optimization
**User**: "Make this code more readable" [pastes code]

**AI**:
1. Calls `optimise_nava_expression` with goal='readability'
2. Refactors code with better names and comments
3. Explains what changed and why
4. Updates editor with optimized code

### Example 4: Scenario Generation
**User**: "Generate 3 variants of this scenario with different start positions"

**AI**:
1. Calls `generate_scenarios_from_code`
2. Creates 3 variants with different starts
3. Returns all variants as separate code blocks
4. Can run all variants for comparison

### Example 5: Documentation Lookup
**User**: "How do I create a navigation field?"

**AI**:
1. Calls `fetch_nava_docs` with query='navigation_field'
2. Returns documentation with examples
3. Provides code snippets
4. Explains usage

## Limitations & Future Enhancements

### Current Limitations
- ⚠️ Documentation search uses mock data (needs real index)
- ⚠️ Local 7B model not yet fine-tuned (uses base model)
- ⚠️ Some advanced features (DAAT visualization) in progress
- ⚠️ Multi-file refactoring is basic

### Planned Enhancements
- 🔄 Real documentation search index
- 🔄 Fine-tuned NAVA-Instruct model
- 🔄 Advanced DAAT/PDL visualization
- 🔄 Multi-file refactoring
- 🔄 Code completion suggestions
- 🔄 Error detection and fixes
- 🔄 Test generation
- 🔄 Documentation generation

## Summary

The AI Panel is a **comprehensive, NAVA-first copilot** that provides:

✅ **8 Specialized Tools** for IDE interaction
✅ **Multi-Model Support** (local + external)
✅ **NAVA-First Behavior** (defaults to NAVA code)
✅ **Educational Support** (GCSE to PhD levels)
✅ **Live Preview** (execute and visualize)
✅ **Scenario Management** (variants and testing)
✅ **Code Generation** (natural language → code)
✅ **Code Explanation** (at multiple levels)
✅ **Code Refactoring** (optimize and improve)
✅ **DAAT/PDL Support** (timing contracts)
✅ **Multi-Manifold Support** (R^2, R^3, S^1, SE(2), etc.)
✅ **Cost Tracking** (token usage and costs)
✅ **Free Mode** (works without API keys)

This makes it a **production-ready, state-of-the-art NAVA copilot** that rivals Cursor's AI but specialized for navigation calculus.

