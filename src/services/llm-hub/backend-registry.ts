/**
 * Backend Registry
 * 
 * Manages all LLM backends and provides unified access.
 * The IDE and agents never talk to providers directly - only through this registry.
 */

import type { LLMBackend, BackendConfig, ChatRequest, ChatResponse } from './types';
import { OpenAIBackend } from './backends/openai-backend';
import { OpenRouterBackend } from './backends/openrouter-backend';
import { NAVALocalBackend } from './backends/nava-local-backend';
import { HuggingFaceBackend } from './backends/huggingface-backend';
import { GeminiBackend } from './backends/gemini-backend';
import { buildNAVAMessages } from './nava-system-prompt';
import type { Context } from './nava-system-prompt';

class BackendRegistry {
  private backends: Map<string, LLMBackend> = new Map();
  private defaultBackendId: string = 'nava-local';

  /**
   * Register a backend
   */
  register(backend: LLMBackend): void {
    this.backends.set(backend.id, backend);
  }

  /**
   * Get a backend by ID
   */
  getBackend(backendId: string): LLMBackend | undefined {
    return this.backends.get(backendId);
  }

  /**
   * Get backend by provider and model
   */
  getBackendByConfig(config: BackendConfig): LLMBackend | undefined {
    const id = `${config.provider}:${config.model}`;
    return this.backends.get(id);
  }

  /**
   * Get all available backends
   */
  getAllBackends(): LLMBackend[] {
    return Array.from(this.backends.values());
  }

  /**
   * Get available backends (configured and ready)
   */
  getAvailableBackends(): LLMBackend[] {
    return Array.from(this.backends.values()).filter(b => b.isAvailable());
  }

  /**
   * Get default backend
   */
  getDefaultBackend(): LLMBackend | undefined {
    const defaultBackend = this.backends.get(this.defaultBackendId);
    if (defaultBackend?.isAvailable()) {
      return defaultBackend;
    }
    
    // Fallback to first available backend
    return this.getAvailableBackends()[0];
  }

  /**
   * Set default backend
   */
  setDefaultBackend(backendId: string): void {
    if (this.backends.has(backendId)) {
      this.defaultBackendId = backendId;
      localStorage.setItem('nava_default_backend', backendId);
    }
  }

  /**
   * Initialize backends from configuration
   */
  initialize(configs: BackendConfig[]): void {
    // Clear existing backends
    this.backends.clear();

    // Register NAVA Local (always available, connects to fine-tuned model server)
    const navaLocalUrl = import.meta.env.VITE_NAVA_LOCAL_URL || 'http://localhost:8080';
    const navaLocal = new NAVALocalBackend({ baseUrl: navaLocalUrl });
    this.register(navaLocal);

    // Register configured backends
    for (const config of configs) {
      try {
        let backend: LLMBackend;

        switch (config.provider) {
          case 'openai':
            backend = new OpenAIBackend(config);
            break;
          case 'openrouter':
            backend = new OpenRouterBackend(config);
            break;
          case 'gemini':
            backend = new GeminiBackend(config);
            break;
          case 'huggingface':
            backend = new HuggingFaceBackend(config);
            break;
          case 'nava-local':
            backend = new NAVALocalBackend(config);
            break;
          default:
            console.warn(`Unknown backend provider: ${config.provider}`);
            continue;
        }

        if (backend.isAvailable()) {
          this.register(backend);
        }
      } catch (error) {
        console.error(`Failed to initialize backend ${config.provider}:${config.model}:`, error);
      }
    }

    // Restore default backend from localStorage
    const savedDefault = localStorage.getItem('nava_default_backend');
    if (savedDefault && this.backends.has(savedDefault)) {
      this.defaultBackendId = savedDefault;
    }
  }

  /**
   * Chat with a backend (with NAVA system prompt wrapper)
   */
  async chat(
    userPrompt: string,
    context?: Context,
    backendId?: string,
    tools?: import('./types').ToolDefinition[]
  ): Promise<ChatResponse> {
    const backend = backendId 
      ? this.getBackend(backendId)
      : this.getDefaultBackend();

    if (!backend) {
      throw new Error('No available backend found');
    }

    // Build messages with NAVA system prompt
    const messages = buildNAVAMessages(userPrompt, context);

    // Create chat request with tools
    const request: ChatRequest = {
      messages,
      temperature: 0.2,
      maxTokens: 2000,
      tools,
      toolChoice: tools && tools.length > 0 ? 'auto' : 'none',
    };

    // Call backend
    const response = await backend.chat(request);
    
    // If backend returned tool calls, execute them
    if (response.toolCalls && response.toolCalls.length > 0) {
      // This would be handled by the caller (AI panel) to execute tools
      // and send results back to the model
    }
    
    return response;
  }

  /**
   * Stream chat with a backend
   */
  async *streamChat(
    userPrompt: string,
    context?: Context,
    backendId?: string
  ): AsyncIterable<import('./types').ChatChunk> {
    const backend = backendId 
      ? this.getBackend(backendId)
      : this.getDefaultBackend();

    if (!backend) {
      throw new Error('No available backend found');
    }

    if (!backend.streamChat) {
      // Fallback to non-streaming
      const response = await this.chat(userPrompt, context, backendId);
      yield { text: response.text, done: true, usage: response.usage };
      return;
    }

    // Build messages with NAVA system prompt
    const messages = buildNAVAMessages(userPrompt, context);

    // Create chat request
    const request: ChatRequest = {
      messages,
      temperature: 0.2,
      maxTokens: 2000,
      stream: true,
    };

    // Stream from backend
    yield* backend.streamChat(request);
  }
}

export const backendRegistry = new BackendRegistry();

