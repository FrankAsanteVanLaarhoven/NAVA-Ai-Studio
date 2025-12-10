/**
 * OpenAI Backend Adapter
 * Wraps OpenAI Chat Completions API
 */

import type { LLMBackend, ChatRequest, ChatResponse, BackendConfig, Capability } from '../types';

export class OpenAIBackend implements LLMBackend {
  id: string;
  displayName: string;
  capabilities: Capability[] = ['code', 'maths', 'navcalc', 'chat', 'reasoning'];
  
  private apiKey: string;
  private model: string;
  private baseUrl: string;

  constructor(config: BackendConfig) {
    if (!config.apiKey) {
      throw new Error('OpenAI backend requires API key');
    }
    
    this.apiKey = config.apiKey;
    this.model = config.model || 'gpt-4o-mini';
    this.baseUrl = config.baseUrl || 'https://api.openai.com/v1';
    this.id = `openai:${this.model}`;
    this.displayName = `OpenAI ${this.model}`;
  }

  isAvailable(): boolean {
    return !!this.apiKey && this.apiKey.length > 0;
  }

  getConfig(): BackendConfig {
    return {
      provider: 'openai',
      model: this.model,
      apiKey: this.apiKey,
      baseUrl: this.baseUrl,
      enabled: this.isAvailable(),
    };
  }

  async chat(request: ChatRequest): Promise<ChatResponse> {
    if (!this.isAvailable()) {
      throw new Error('OpenAI backend is not configured');
    }

    const response = await fetch(`${this.baseUrl}/chat/completions`, {
      method: 'POST',
      headers: {
        'Authorization': `Bearer ${this.apiKey}`,
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        model: this.model,
        messages: request.messages.map(msg => ({
          role: msg.role,
          content: msg.content,
          name: msg.name,
          tool_calls: msg.tool_calls,
          tool_call_id: msg.tool_call_id,
        })),
        temperature: request.temperature ?? 0.2,
        max_tokens: request.maxTokens,
        top_p: request.topP,
        stream: request.stream ?? false,
        tools: request.tools,
        tool_choice: request.toolChoice,
      }),
    });

    if (!response.ok) {
      const error = await response.json().catch(() => ({ error: { message: response.statusText } }));
      throw new Error(`OpenAI API error: ${error.error?.message || response.statusText}`);
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
      toolCalls: choice.message.tool_calls,
    };
  }

  async *streamChat(request: ChatRequest): AsyncIterable<import('../types').ChatChunk> {
    if (!this.isAvailable()) {
      throw new Error('OpenAI backend is not configured');
    }

    const response = await fetch(`${this.baseUrl}/chat/completions`, {
      method: 'POST',
      headers: {
        'Authorization': `Bearer ${this.apiKey}`,
        'Content-Type': 'application/json',
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
      throw new Error(`OpenAI API error: ${response.statusText}`);
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

