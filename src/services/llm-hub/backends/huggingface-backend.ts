/**
 * HuggingFace Backend Adapter
 * For Inference Endpoints / text-generation-inference
 */

import type { LLMBackend, ChatRequest, ChatResponse, BackendConfig, Capability } from '../types';

export class HuggingFaceBackend implements LLMBackend {
  id: string;
  displayName: string;
  capabilities: Capability[] = ['code', 'maths', 'chat'];
  
  private apiKey?: string;
  private model: string;
  private baseUrl: string;

  constructor(config: BackendConfig) {
    this.apiKey = config.apiKey;
    this.model = config.model || 'mistralai/Mistral-7B-Instruct-v0.2';
    this.baseUrl = config.baseUrl || 'https://api-inference.huggingface.co/models';
    this.id = `huggingface:${this.model}`;
    this.displayName = `HuggingFace ${this.model}`;
  }

  isAvailable(): boolean {
    // HuggingFace can work without API key for some models
    return true;
  }

  getConfig(): BackendConfig {
    return {
      provider: 'huggingface',
      model: this.model,
      apiKey: this.apiKey,
      baseUrl: this.baseUrl,
      enabled: true,
    };
  }

  async chat(request: ChatRequest): Promise<ChatResponse> {
    // Convert messages to prompt format (HuggingFace uses different format)
    const prompt = this.messagesToPrompt(request.messages);

    const headers: Record<string, string> = {
      'Content-Type': 'application/json',
    };

    if (this.apiKey) {
      headers['Authorization'] = `Bearer ${this.apiKey}`;
    }

    const response = await fetch(`${this.baseUrl}/${this.model}`, {
      method: 'POST',
      headers,
      body: JSON.stringify({
        inputs: prompt,
        parameters: {
          max_new_tokens: request.maxTokens || 500,
          temperature: request.temperature ?? 0.7,
          top_p: request.topP ?? 0.9,
          return_full_text: false,
        },
      }),
    });

    if (!response.ok) {
      const error = await response.text();
      throw new Error(`HuggingFace API error: ${error}`);
    }

    const json = await response.json();
    const generatedText = Array.isArray(json) ? json[0]?.generated_text : json.generated_text;

    return {
      text: generatedText || '',
      usage: {
        inputTokens: 0, // HuggingFace doesn't always provide usage
        outputTokens: 0,
      },
    };
  }

  private messagesToPrompt(messages: import('../types').ChatMessage[]): string {
    // Simple conversion - in production, use proper chat template
    return messages
      .map(msg => {
        if (msg.role === 'system') {
          return `System: ${msg.content}`;
        } else if (msg.role === 'user') {
          return `User: ${msg.content}`;
        } else if (msg.role === 'assistant') {
          return `Assistant: ${msg.content}`;
        }
        return '';
      })
      .join('\n\n') + '\n\nAssistant:';
  }
}

