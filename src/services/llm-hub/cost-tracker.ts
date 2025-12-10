/**
 * Cost Tracker
 * 
 * Tracks token usage and estimates costs for different backends.
 */

import type { Usage } from './types';

export interface PricePerK {
  input: number;  // Price per 1K input tokens
  output: number; // Price per 1K output tokens
}

// Price database (in USD per 1K tokens)
// These are approximate - update with actual pricing
const PRICE_DATABASE: Record<string, PricePerK> = {
  // OpenAI
  'openai:gpt-4o': { input: 2.50 / 1000, output: 10.00 / 1000 },
  'openai:gpt-4o-mini': { input: 0.15 / 1000, output: 0.60 / 1000 },
  'openai:gpt-4-turbo': { input: 10.00 / 1000, output: 30.00 / 1000 },
  'openai:gpt-3.5-turbo': { input: 0.50 / 1000, output: 1.50 / 1000 },
  
  // OpenRouter (approximate, varies by model)
  'openrouter:openai/gpt-4o': { input: 2.50 / 1000, output: 10.00 / 1000 },
  'openrouter:openai/gpt-4o-mini': { input: 0.15 / 1000, output: 0.60 / 1000 },
  'openrouter:anthropic/claude-3.5-sonnet': { input: 3.00 / 1000, output: 15.00 / 1000 },
  'openrouter:google/gemini-3-pro': { input: 0.50 / 1000, output: 1.50 / 1000 },
  
  // NAVA Local (free)
  'nava-local': { input: 0, output: 0 },
  'nava-local:nava-7b': { input: 0, output: 0 },
  
  // HuggingFace (free for public models)
  'huggingface': { input: 0, output: 0 },
};

export class CostTracker {
  private totalUsage: Map<string, Usage> = new Map();
  private totalCost: Map<string, number> = new Map();

  /**
   * Estimate cost for a backend usage
   */
  estimateCost(backendId: string, usage: Usage): number | null {
    const prices = PRICE_DATABASE[backendId];
    if (!prices) {
      return null; // Unknown pricing
    }

    const inputCost = (usage.inputTokens / 1000) * prices.input;
    const outputCost = (usage.outputTokens / 1000) * prices.output;
    return inputCost + outputCost;
  }

  /**
   * Track usage and cost
   */
  track(backendId: string, usage: Usage): void {
    // Update total usage
    const currentUsage = this.totalUsage.get(backendId) || { inputTokens: 0, outputTokens: 0 };
    this.totalUsage.set(backendId, {
      inputTokens: currentUsage.inputTokens + usage.inputTokens,
      outputTokens: currentUsage.outputTokens + usage.outputTokens,
      totalTokens: (currentUsage.totalTokens || 0) + (usage.totalTokens || usage.inputTokens + usage.outputTokens),
    });

    // Update total cost
    const cost = this.estimateCost(backendId, usage);
    if (cost !== null) {
      const currentCost = this.totalCost.get(backendId) || 0;
      this.totalCost.set(backendId, currentCost + cost);
    }
  }

  /**
   * Get total usage for a backend
   */
  getTotalUsage(backendId: string): Usage | undefined {
    return this.totalUsage.get(backendId);
  }

  /**
   * Get total cost for a backend
   */
  getTotalCost(backendId: string): number | null {
    const cost = this.totalCost.get(backendId);
    return cost !== undefined ? cost : null;
  }

  /**
   * Get all usage stats
   */
  getAllStats(): Array<{ backendId: string; usage: Usage; cost: number | null }> {
    return Array.from(this.totalUsage.entries()).map(([backendId, usage]) => ({
      backendId,
      usage,
      cost: this.getTotalCost(backendId),
    }));
  }

  /**
   * Reset stats
   */
  reset(): void {
    this.totalUsage.clear();
    this.totalCost.clear();
  }

  /**
   * Format cost for display
   */
  formatCost(cost: number | null): string {
    if (cost === null) return 'N/A';
    if (cost === 0) return 'Free';
    if (cost < 0.01) return `$${(cost * 1000).toFixed(2)}m`; // millicents
    return `$${cost.toFixed(4)}`;
  }

  /**
   * Format usage for display
   */
  formatUsage(usage: Usage): string {
    return `${usage.inputTokens.toLocaleString()} in / ${usage.outputTokens.toLocaleString()} out`;
  }
}

export const costTracker = new CostTracker();

