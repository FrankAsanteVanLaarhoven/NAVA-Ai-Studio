/**
 * Backend Initializer
 * 
 * Bridges model-config-service with backend-registry
 * Converts model configurations into backend configs
 */

import { backendRegistry } from './backend-registry';
import { modelConfigService } from '../model-config-service';
import type { BackendConfig } from './types';

/**
 * Initialize all backends from model configuration
 */
export function initializeBackends(): void {
  const backendConfigs: BackendConfig[] = [];

  // Get API keys from model config service
  const geminiKey = modelConfigService.getProviderApiKey('gemini') || import.meta.env.VITE_GEMINI_API_KEY;
  const openRouterKey = modelConfigService.getProviderApiKey('openrouter') || import.meta.env.VITE_OPENROUTER_API_KEY;
  const openAIKey = modelConfigService.getProviderApiKey('openai') || import.meta.env.VITE_OPENAI_API_KEY;

  // Get enabled models
  const models = modelConfigService.getEnabledModels();

  // Create backend configs from models
  for (const model of models) {
    if (model.provider === 'gemini' && geminiKey) {
      // Map Gemini model IDs to backend configs
      const modelId = model.id.replace('gemini/', '').replace('google/', '');
      
      // Some Gemini models are only available via OpenRouter
      if (modelId.includes('gemini-3') || modelId.includes('gemini-2.5')) {
        if (openRouterKey) {
          backendConfigs.push({
            provider: 'openrouter',
            model: `google/${modelId}`,
            apiKey: openRouterKey,
            enabled: true,
          });
        }
      } else {
        // Direct Gemini API models
        const geminiModelName = modelId === 'gemini-1.5-pro-latest' ? 'gemini-1.5-pro' : 
                                modelId === 'gemini-1.5-flash-latest' ? 'gemini-1.5-flash' :
                                modelId === 'gemini-2.0-flash-experimental' ? 'gemini-2.0-flash-exp' :
                                modelId;
        backendConfigs.push({
          provider: 'gemini',
          model: geminiModelName,
          apiKey: geminiKey,
          baseUrl: 'https://generativelanguage.googleapis.com/v1beta/models',
          enabled: true,
        });
      }
    } else if (model.provider === 'openrouter' && openRouterKey) {
      backendConfigs.push({
        provider: 'openrouter',
        model: model.id,
        apiKey: openRouterKey,
        enabled: true,
      });
    } else if (model.provider === 'openai' && openAIKey) {
      backendConfigs.push({
        provider: 'openai',
        model: model.id,
        apiKey: openAIKey,
        enabled: true,
      });
    } else if (model.provider === 'huggingface') {
      // HuggingFace models - extract model name from ID (format: huggingface:model-name)
      const modelName = model.id.replace('huggingface:', '');
      backendConfigs.push({
        provider: 'huggingface',
        model: modelName,
        enabled: true,
      });
    }
  }

  // Always add NAVA Local backend (free, always available)
  backendConfigs.push({
    provider: 'nava-local',
    model: 'nava-7b',
    enabled: true,
  });

  // Always add HuggingFace backend (free, always available, no API key needed)
  backendConfigs.push({
    provider: 'huggingface',
    model: 'mistralai/Mistral-7B-Instruct-v0.2',
    enabled: true,
  });

  // Initialize registry
  backendRegistry.initialize(backendConfigs);
  
  // Log initialization for debugging
  console.log('[Backend Initializer] Initialized backends:', backendConfigs.map(c => `${c.provider}:${c.model}`));
  console.log('[Backend Initializer] Available backends:', backendRegistry.getAvailableBackends().map(b => b.id));
}

/**
 * Get backend ID from model ID and provider
 */
export function getBackendIdFromModel(modelId: string, provider: string): string {
  // Handle HuggingFace models (format: huggingface:model-name)
  if (provider === 'huggingface' || modelId.startsWith('huggingface:')) {
    const modelName = modelId.replace('huggingface:', '');
    return `huggingface:${modelName}`;
  }
  
  // Handle OpenRouter provider models (like google/gemini-2.5-pro)
  if (provider === 'openrouter' || modelId.startsWith('google/') || modelId.startsWith('openai/') || modelId.startsWith('anthropic/')) {
    return `openrouter:${modelId}`;
  }
  
  if (provider === 'gemini') {
    const cleanId = modelId.replace('gemini/', '').replace('google/', '');
    if (cleanId.includes('gemini-3') || cleanId.includes('gemini-2.5')) {
      return `openrouter:google/${cleanId}`;
    }
    // Map to Gemini backend
    const geminiModelName = cleanId === 'gemini-1.5-pro-latest' ? 'gemini-1.5-pro' : 
                            cleanId === 'gemini-1.5-flash-latest' ? 'gemini-1.5-flash' :
                            cleanId === 'gemini-2.0-flash-experimental' ? 'gemini-2.0-flash-exp' :
                            cleanId;
    return `gemini:${geminiModelName}`;
  }
  
  return `${provider}:${modelId}`;
}

