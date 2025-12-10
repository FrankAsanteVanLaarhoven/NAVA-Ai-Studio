/**
 * OpenRouter Backend Adapter
 * Wraps OpenRouter API for accessing multiple models
 */

import type { LLMBackend, ChatRequest, ChatResponse, BackendConfig, Capability } from '../types';

export class OpenRouterBackend implements LLMBackend {
  id: string;
  displayName: string;
  capabilities: Capability[] = ['code', 'maths', 'navcalc', 'chat', 'reasoning'];
  
  private apiKey: string;
  private model: string;
  private baseUrl: string;

  constructor(config: BackendConfig) {
    if (!config.apiKey) {
      throw new Error('OpenRouter backend requires API key');
    }
    
    this.apiKey = config.apiKey;
    this.model = config.model || 'openai/gpt-4o';
    this.baseUrl = config.baseUrl || 'https://openrouter.ai/api/v1';
    this.id = `openrouter:${this.model}`;
    this.displayName = `OpenRouter ${this.model}`;
  }

  isAvailable(): boolean {
    return !!this.apiKey && this.apiKey.length > 0;
  }

  getConfig(): BackendConfig {
    return {
      provider: 'openrouter',
      model: this.model,
      apiKey: this.apiKey,
      baseUrl: this.baseUrl,
      enabled: this.isAvailable(),
    };
  }

  async chat(request: ChatRequest): Promise<ChatResponse> {
    if (!this.isAvailable()) {
      throw new Error('OpenRouter backend is not configured');
    }

    const response = await fetch(`${this.baseUrl}/chat/completions`, {
      method: 'POST',
      headers: {
        'Authorization': `Bearer ${this.apiKey}`,
        'Content-Type': 'application/json',
        'HTTP-Referer': window.location.origin, // OpenRouter requires this
        'X-Title': 'NAVA Studio IDE',
      },
      body: JSON.stringify({
        model: this.model,
        messages: request.messages.map(msg => ({
          role: msg.role,
          content: msg.content,
          name: msg.name,
        })),
        temperature: request.temperature ?? 0.2,
        max_tokens: request.maxTokens,
        top_p: request.topP,
        stream: request.stream ?? false,
      }),
    });

    if (!response.ok) {
      const error = await response.json().catch(() => ({ error: { message: response.statusText } }));
      throw new Error(`OpenRouter API error: ${error.error?.message || response.statusText}`);
    }

    const json = await response.json();
    const choice = json.choices[0];

    return {
      text: choice.message.content || '',
      usage: json.usage ? {
        inputTokens: json.usage.prompt_tokens,
        outputTokens: json.usage.completion_tokens,
        totalTokens: json.usage.total_tokens,
      } : undefined,
      finishReason: choice.finish_reason,
    };
  }

  async *streamChat(request: ChatRequest): AsyncIterable<import('../types').ChatChunk> {
    if (!this.isAvailable()) {
      throw new Error('OpenRouter backend is not configured');
    }

    const response = await fetch(`${this.baseUrl}/chat/completions`, {
      method: 'POST',
      headers: {
        'Authorization': `Bearer ${this.apiKey}`,
        'Content-Type': 'application/json',
        'HTTP-Referer': window.location.origin,
        'X-Title': 'NAVA Studio IDE',
      },
      body: JSON.stringify({
        model: this.model,
        messages: request.messages,
        temperature: request.temperature ?? 0.2,
        max_tokens: request.maxTokens,
        stream: true,
      }),
    });

    if (!response.ok) {
      throw new Error(`OpenRouter API error: ${response.statusText}`);
    }

    const reader = response.body?.getReader();
    const decoder = new TextDecoder();

    if (!reader) {
      throw new Error('Failed to get response stream');
    }

    let buffer = '';
    let totalInputTokens = 0;
    let totalOutputTokens = 0;

    try {
      while (true) {
        const { done, value } = await reader.read();
        if (done) break;

        buffer += decoder.decode(value, { stream: true });
        const lines = buffer.split('\n');
        buffer = lines.pop() || '';

        for (const line of lines) {
          if (line.startsWith('data: ')) {
            const data = line.slice(6);
            if (data === '[DONE]') {
              yield { text: '', done: true, usage: { inputTokens: totalInputTokens, outputTokens: totalOutputTokens } };
              return;
            }

            try {
              const json = JSON.parse(data);
              const delta = json.choices[0]?.delta;
              if (delta?.content) {
                yield { text: delta.content, done: false };
              }
              if (json.usage) {
                totalInputTokens = json.usage.prompt_tokens || totalInputTokens;
                totalOutputTokens = json.usage.completion_tokens || totalOutputTokens;
              }
            } catch (e) {
              // Skip invalid JSON
            }
          }
        }
      }
    } finally {
      reader.releaseLock();
    }
  }
}

