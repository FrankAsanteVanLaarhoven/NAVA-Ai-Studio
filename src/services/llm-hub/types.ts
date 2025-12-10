/**
 * LLM Hub - Unified Interface for All LLM Backends
 * 
 * Mental Model: The IDE has a single internal interface for all LLM providers.
 * All backends (OpenAI, OpenRouter, NAVA Local, etc.) implement this interface.
 */

export type Capability = 'code' | 'maths' | 'navcalc' | 'chat' | 'reasoning' | 'multimodal';

export interface ChatMessage {
  role: 'system' | 'user' | 'assistant' | 'tool';
  content: string;
  name?: string;
  tool_calls?: ToolCall[];
  tool_call_id?: string;
}

export interface ToolCall {
  id: string;
  type: 'function';
  function: {
    name: string;
    arguments: string; // JSON string
  };
}

export interface ChatRequest {
  messages: ChatMessage[];
  temperature?: number;
  maxTokens?: number;
  topP?: number;
  stream?: boolean;
  tools?: ToolDefinition[];
  toolChoice?: 'auto' | 'none' | { type: 'function'; function: { name: string } };
}

export interface ToolDefinition {
  type: 'function';
  function: {
    name: string;
    description: string;
    parameters: Record<string, any>; // JSON Schema
  };
}

export interface Usage {
  inputTokens: number;
  outputTokens: number;
  totalTokens?: number;
}

export interface ChatResponse {
  text: string;
  usage?: Usage;
  finishReason?: 'stop' | 'length' | 'tool_calls' | 'content_filter';
  toolCalls?: ToolCall[];
}

export interface ChatChunk {
  text: string;
  done: boolean;
  usage?: Usage;
}

export interface BackendConfig {
  provider: string;
  model: string;
  apiKey?: string;
  baseUrl?: string;
  enabled?: boolean;
}

/**
 * Unified LLM Backend Interface
 * 
 * All LLM providers (OpenAI, OpenRouter, NAVA Local, etc.) implement this interface.
 * The IDE and agents never talk to providers directly - only through this interface.
 */
export interface LLMBackend {
  /** Unique identifier: "nava-local", "openai:gpt-4.1", "openrouter:sonnet" */
  id: string;
  
  /** Display name for UI */
  displayName: string;
  
  /** Capabilities this backend supports */
  capabilities: Capability[];
  
  /** Chat completion */
  chat(request: ChatRequest): Promise<ChatResponse>;
  
  /** Streaming chat (optional) */
  streamChat?(request: ChatRequest): AsyncIterable<ChatChunk>;
  
  /** Check if backend is available/configured */
  isAvailable(): boolean;
  
  /** Get backend configuration */
  getConfig(): BackendConfig;
}

