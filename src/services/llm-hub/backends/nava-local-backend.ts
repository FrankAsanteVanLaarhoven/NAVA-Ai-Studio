/**
 * NAVA Local Backend
 * 
 * Connects to a locally served fine-tuned NAVA-Instruct model.
 * The model is served via vLLM or HuggingFace at localhost:8080.
 */

import type { LLMBackend, ChatRequest, ChatResponse, BackendConfig, Capability } from '../types';
import { buildNAVAMessages } from '../nava-system-prompt';

export class NAVALocalBackend implements LLMBackend {
  id: string = 'nava-local';
  displayName: string = 'NAVA Local (7B Fine-tuned)';
  capabilities: Capability[] = ['code', 'navcalc', 'chat', 'reasoning'];
  
  private baseUrl: string;
  private apiKey?: string;

  constructor(config?: { baseUrl?: string; apiKey?: string }) {
    this.baseUrl = config?.baseUrl || 'http://localhost:8080';
    this.apiKey = config?.apiKey;
  }

  async chat(request: ChatRequest): Promise<ChatResponse> {
    try {
      // Format messages for the local model
      // The model expects instruction format
      const formattedMessages = this.formatMessages(request.messages);
      
      // Call local model server
      const response = await fetch(`${this.baseUrl}/v1/chat/completions`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          ...(this.apiKey && { 'Authorization': `Bearer ${this.apiKey}` }),
        },
        body: JSON.stringify({
          messages: formattedMessages,
          temperature: request.temperature || 0.2,
          max_tokens: request.maxTokens || 512,
        }),
      });

      if (!response.ok) {
        const errorText = await response.text();
        throw new Error(`NAVA Local API error: ${response.status} - ${errorText}`);
      }

      const data = await response.json();
      
      // Parse response
      const text = data.choices?.[0]?.message?.content || data.choices?.[0]?.text || '';
      const usage = data.usage ? {
        inputTokens: data.usage.prompt_tokens || 0,
        outputTokens: data.usage.completion_tokens || 0,
        totalTokens: data.usage.total_tokens || 0,
      } : undefined;

      // Handle tool calls if present (local model may return tool calls)
      const toolCalls = data.choices?.[0]?.message?.tool_calls || undefined;

      return {
        text,
        usage,
        finishReason: data.choices?.[0]?.finish_reason || 'stop',
        model: 'nava-instruct-7b',
        toolCalls,
      };
    } catch (error: any) {
      console.error('[NAVA Local Backend] Error:', error);
      
      // Provide helpful error message for connection failures
      if (error.message?.includes('Failed to fetch') || 
          error.message?.includes('NetworkError') ||
          error.message?.includes('fetch failed')) {
        throw new Error(
          `NAVA Local model server is not running.\n\n` +
          `To start it:\n` +
          `1. Open terminal\n` +
          `2. Run: cd "/Users/frankvanlaarhoven/Desktop/NAVA Studio IDE" && ./scripts/start_nava_model.sh\n` +
          `3. Wait for "Server running on http://localhost:8080"\n` +
          `4. Try again\n\n` +
          `Note: You need to train the model first using NAVA_Instruct_Model_Training.ipynb`
        );
      }
      
      throw new Error(`NAVA Local backend error: ${error.message}`);
    }
  }

  /**
   * Format messages for local model
   * The fine-tuned model uses Llama chat template, so we pass messages as-is
   * The server will handle chat template formatting
   */
  private formatMessages(messages: ChatRequest['messages']): Array<{ role: string; content: string }> {
    const formatted: Array<{ role: string; content: string }> = [];
    
    for (const msg of messages) {
      // Keep original roles - the server will use chat template
      if (msg.role === 'system' || msg.role === 'user' || msg.role === 'assistant') {
        formatted.push({
          role: msg.role,
          content: msg.content,
        });
      } else if (msg.role === 'tool') {
        // Tool results can be included as user messages
        formatted.push({
          role: 'user',
          content: `Tool result: ${msg.content}`,
        });
      }
    }
    
    return formatted;
  }

  isAvailable(): boolean {
    // We can't do async checks here, so we return true
    // The actual availability will be checked when chat() is called
    // and will provide a helpful error message if the server is down
    return true;
  }

  getConfig(): BackendConfig {
    return {
      provider: 'nava-local',
      model: 'nava-instruct-7b',
      baseUrl: this.baseUrl,
      enabled: true,
    };
  }
}
