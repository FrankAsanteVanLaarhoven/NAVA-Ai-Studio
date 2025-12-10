/**
 * Gemini Backend Adapter
 * Wraps Google Gemini API
 */

import type { LLMBackend, ChatRequest, ChatResponse, BackendConfig, Capability } from '../types';

export class GeminiBackend implements LLMBackend {
  id: string;
  displayName: string;
  capabilities: Capability[] = ['code', 'maths', 'navcalc', 'chat', 'multimodal'];
  
  private apiKey: string;
  private model: string;
  private baseUrl: string;

  constructor(config: BackendConfig) {
    if (!config.apiKey) {
      throw new Error('Gemini backend requires API key');
    }
    
    this.apiKey = config.apiKey;
    this.model = config.model || 'gemini-1.5-pro';
    this.baseUrl = config.baseUrl || 'https://generativelanguage.googleapis.com/v1beta/models';
    this.id = `gemini:${this.model}`;
    this.displayName = `Gemini ${this.model}`;
  }

  isAvailable(): boolean {
    return !!this.apiKey && this.apiKey.length > 0;
  }

  getConfig(): BackendConfig {
    return {
      provider: 'gemini',
      model: this.model,
      apiKey: this.apiKey,
      baseUrl: this.baseUrl,
      enabled: this.isAvailable(),
    };
  }

  async chat(request: ChatRequest): Promise<ChatResponse> {
    if (!this.isAvailable()) {
      throw new Error('Gemini backend is not configured');
    }

    // Convert messages to Gemini format
    const contents = this.messagesToGeminiFormat(request.messages);

    const response = await fetch(`${this.baseUrl}/${this.model}:generateContent?key=${this.apiKey}`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        contents,
        generationConfig: {
          temperature: request.temperature ?? 0.2,
          maxOutputTokens: request.maxTokens,
          topP: request.topP,
        },
      }),
    });

    if (!response.ok) {
      const error = await response.json().catch(() => ({ error: { message: response.statusText } }));
      throw new Error(`Gemini API error: ${error.error?.message || response.statusText}`);
    }

    const json = await response.json();
    const candidate = json.candidates?.[0];
    const content = candidate?.content?.parts?.[0]?.text || '';

    return {
      text: content,
      usage: json.usageMetadata ? {
        inputTokens: json.usageMetadata.promptTokenCount || 0,
        outputTokens: json.usageMetadata.candidatesTokenCount || 0,
        totalTokens: json.usageMetadata.totalTokenCount || 0,
      } : undefined,
      finishReason: candidate?.finishReason === 'STOP' ? 'stop' : undefined,
    };
  }

  private messagesToGeminiFormat(messages: import('../types').ChatMessage[]): any[] {
    // Gemini uses a different format - convert from standard chat format
    const contents: any[] = [];
    
    for (const msg of messages) {
      if (msg.role === 'system') {
        // Gemini doesn't have system messages, prepend to first user message
        if (contents.length === 0 || contents[contents.length - 1].role !== 'user') {
          contents.push({
            role: 'user',
            parts: [{ text: `System: ${msg.content}` }],
          });
        } else {
          // Append to last user message
          const lastContent = contents[contents.length - 1];
          lastContent.parts[0].text = `${lastContent.parts[0].text}\n\nSystem: ${msg.content}`;
        }
      } else if (msg.role === 'user') {
        contents.push({
          role: 'user',
          parts: [{ text: msg.content }],
        });
      } else if (msg.role === 'assistant') {
        contents.push({
          role: 'model',
          parts: [{ text: msg.content }],
        });
      }
    }

    return contents;
  }
}

