/**
 * Model Configuration Service
 * Manages AI model configurations, API keys, and custom models
 */

export type ModelProvider = 'gemini' | 'openrouter' | 'openai' | 'anthropic' | 'custom' | 'mcp';

export interface CustomModel {
  id: string;
  name: string;
  description: string;
  provider: ModelProvider;
  apiEndpoint: string;
  apiKey?: string; // Stored separately for security
  contextLength: number;
  enabled: boolean;
  isCustom: true;
  mcpConnectionId?: string; // For MCP-enabled models
  mcpServiceType?: string; // GitHub, Slack, Linear, etc.
}

export interface BuiltInModel {
  id: string;
  name: string;
  description: string;
  provider: ModelProvider;
  contextLength: number;
  enabled: boolean;
  isCustom: false;
  apiEndpoint?: string;
}

export type AIModel = BuiltInModel | CustomModel;

export interface ProviderConfig {
  provider: ModelProvider;
  apiKey: string;
  enabled: boolean;
  customEndpoint?: string;
}

class ModelConfigService {
  private readonly STORAGE_KEY = 'nava_model_config';
  private readonly API_KEYS_STORAGE_KEY = 'nava_api_keys';

  /**
   * Get all configured models (built-in + custom), sorted alphabetically
   */
  getModels(): AIModel[] {
    const config = this.getConfig();
    const models = config.models;
    // Sort alphabetically by name (case-insensitive, ignoring emojis)
    return models.sort((a, b) => {
      // Remove emojis and special characters for sorting
      const nameA = a.name.replace(/[^\w\s]/g, '').toLowerCase();
      const nameB = b.name.replace(/[^\w\s]/g, '').toLowerCase();
      return nameA.localeCompare(nameB);
    });
  }

  /**
   * Get enabled models only, sorted alphabetically by name
   */
  getEnabledModels(): AIModel[] {
    const models = this.getModels().filter(m => m.enabled);
    // Sort alphabetically by name (case-insensitive, ignoring emojis)
    return models.sort((a, b) => {
      // Remove emojis and special characters for sorting
      const nameA = a.name.replace(/[^\w\s]/g, '').toLowerCase();
      const nameB = b.name.replace(/[^\w\s]/g, '').toLowerCase();
      return nameA.localeCompare(nameB);
    });
  }

  /**
   * Get custom models
   */
  getCustomModels(): CustomModel[] {
    return this.getModels().filter((m): m is CustomModel => m.isCustom);
  }

  /**
   * Add a custom model
   */
  addCustomModel(model: Omit<CustomModel, 'id' | 'isCustom'>): CustomModel {
    const config = this.getConfig();
    const newModel: CustomModel = {
      ...model,
      id: `custom-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`,
      isCustom: true,
    };
    config.models.push(newModel);
    this.saveConfig(config);
    return newModel;
  }

  /**
   * Update a model
   */
  updateModel(modelId: string, updates: Partial<AIModel>): AIModel | null {
    const config = this.getConfig();
    const index = config.models.findIndex(m => m.id === modelId);
    if (index === -1) return null;
    
    config.models[index] = { ...config.models[index], ...updates };
    this.saveConfig(config);
    return config.models[index];
  }

  /**
   * Delete a custom model
   */
  deleteModel(modelId: string): boolean {
    const config = this.getConfig();
    const index = config.models.findIndex(m => m.id === modelId && m.isCustom);
    if (index === -1) return false;
    
    config.models.splice(index, 1);
    this.saveConfig(config);
    return true;
  }

  /**
   * Get provider API key
   */
  getProviderApiKey(provider: ModelProvider): string {
    const keys = this.getApiKeys();
    return keys[provider] || '';
  }

  /**
   * Set provider API key
   */
  setProviderApiKey(provider: ModelProvider, apiKey: string): void {
    const keys = this.getApiKeys();
    keys[provider] = apiKey;
    this.saveApiKeys(keys);
  }

  /**
   * Get provider configuration
   */
  getProviderConfig(provider: ModelProvider): ProviderConfig {
    const config = this.getConfig();
    const providerConfig = config.providers[provider] || {
      provider,
      apiKey: this.getProviderApiKey(provider),
      enabled: true,
    };
    return {
      ...providerConfig,
      apiKey: this.getProviderApiKey(provider), // Always get from secure storage
    };
  }

  /**
   * Update provider configuration
   */
  updateProviderConfig(provider: ModelProvider, config: Partial<ProviderConfig>): void {
    const fullConfig = this.getConfig();
    fullConfig.providers[provider] = {
      ...fullConfig.providers[provider],
      ...config,
    };
    this.saveConfig(fullConfig);
    
    // Save API key separately
    if (config.apiKey !== undefined) {
      this.setProviderApiKey(provider, config.apiKey);
    }
  }

  /**
   * Get default model
   */
  getDefaultModel(): string {
    const config = this.getConfig();
    return config.defaultModel || 'gemini-1.5-pro-latest';
  }

  /**
   * Set default model
   */
  setDefaultModel(modelId: string): void {
    const config = this.getConfig();
    config.defaultModel = modelId;
    this.saveConfig(config);
  }

  /**
   * Initialize with default models
   */
  private getConfig(): {
    models: AIModel[];
    providers: Record<ModelProvider, ProviderConfig>;
    defaultModel: string;
  } {
    const stored = localStorage.getItem(this.STORAGE_KEY);
    if (stored) {
      try {
        const parsed = JSON.parse(stored);
        // Remove duplicates by ID
        if (parsed.models && Array.isArray(parsed.models)) {
          const seenIds = new Set<string>();
          parsed.models = parsed.models.filter((m: AIModel) => {
            if (seenIds.has(m.id)) {
              return false; // Duplicate, remove it
            }
            seenIds.add(m.id);
            return true; // Keep it
          });
        }
        
        // If stored config has fewer models than defaults, merge with defaults
        // This ensures new models are always available
        const defaultConfig = this.getDefaultConfig();
        if (parsed.models && parsed.models.length < defaultConfig.models.length) {
          // Merge: keep existing models, add new ones from defaults
          const existingIds = new Set(parsed.models.map((m: AIModel) => m.id));
          const newModels = defaultConfig.models.filter(m => !existingIds.has(m.id));
          parsed.models = [...parsed.models, ...newModels];
          // Update providers if needed
          parsed.providers = { ...defaultConfig.providers, ...parsed.providers };
          this.saveConfig(parsed);
          return parsed;
        }
        
        // Save deduplicated config back to localStorage
        if (parsed.models) {
          this.saveConfig(parsed);
        }
        
        return parsed;
      } catch {
        // Fall through to defaults
      }
    }

    return this.getDefaultConfig();
  }

  private getDefaultConfig(): {
    models: AIModel[];
    providers: Record<ModelProvider, ProviderConfig>;
    defaultModel: string;
  } {

    // Default models - Comprehensive list including all major models
    const defaultModels: BuiltInModel[] = [
      // NAVA Local Model (Fine-tuned 7B)
      {
        id: 'nava-local',
        name: '🤖 NAVA Local (7B Fine-tuned)',
        description: 'Local NAVA-fluent model fine-tuned on NAVA-Instruct dataset. Connects to localhost:8080',
        provider: 'openrouter', // Use openrouter provider type for compatibility
        contextLength: 2048,
        enabled: true,
        isCustom: false,
      },
      // Gemini Models
      {
        id: 'gemini-2.0-flash-experimental',
        name: '✨ Gemini 2.0 Flash Experimental',
        description: 'Google\'s latest experimental multimodal model',
        provider: 'gemini',
        contextLength: 1000000,
        enabled: true,
        isCustom: false,
        apiEndpoint: 'https://generativelanguage.googleapis.com/v1beta/models/gemini-2.0-flash-experimental:generateContent',
      },
      {
        id: 'gemini-1.5-pro-latest',
        name: '⚡ Gemini 1.5 Pro Latest',
        description: 'Google\'s most capable model with 1M context',
        provider: 'gemini',
        contextLength: 1000000,
        enabled: true,
        isCustom: false,
        apiEndpoint: 'https://generativelanguage.googleapis.com/v1beta/models/gemini-1.5-pro-latest:generateContent',
      },
      {
        id: 'google/gemini-3-pro',
        name: '✨ Gemini 3 Pro',
        description: 'Google\'s next-generation AI model',
        provider: 'openrouter',
        contextLength: 2000000,
        enabled: true,
        isCustom: false,
      },
      {
        id: 'google/gemini-2.5-pro',
        name: '⚡ Gemini 2.5 Pro',
        description: 'Enhanced Gemini Pro with improved reasoning',
        provider: 'openrouter',
        contextLength: 1000000,
        enabled: true,
        isCustom: false,
      },
      // OpenAI Models (via OpenRouter)
      {
        id: 'openai/gpt-4o',
        name: '🚀 GPT-4o',
        description: 'OpenAI\'s latest flagship model',
        provider: 'openrouter',
        contextLength: 128000,
        enabled: true,
        isCustom: false,
      },
      {
        id: 'openai/gpt-5-codex',
        name: '🚀 GPT-5 Codex',
        description: 'OpenAI\'s advanced coding model',
        provider: 'openrouter',
        contextLength: 256000,
        enabled: true,
        isCustom: false,
      },
      {
        id: 'openai/gpt-5.1-codex',
        name: '🚀 GPT-5.1 Codex',
        description: 'Enhanced GPT-5 Codex with improved performance',
        provider: 'openrouter',
        contextLength: 256000,
        enabled: true,
        isCustom: false,
      },
      // Anthropic Models (via OpenRouter)
      {
        id: 'anthropic/claude-3.5-sonnet',
        name: '✨ Claude 3.5 Sonnet',
        description: 'Best for coding! Most capable model',
        provider: 'openrouter',
        contextLength: 200000,
        enabled: true,
        isCustom: false,
      },
      {
        id: 'anthropic/claude-sonnet-4.5',
        name: '✨ Sonnet 4.5',
        description: 'Anthropic\'s latest Sonnet model',
        provider: 'openrouter',
        contextLength: 200000,
        enabled: true,
        isCustom: false,
      },
      {
        id: 'anthropic/claude-sonnet-4.5-thinking',
        name: '✨ Sonnet 4.5 Thinking',
        description: 'Sonnet 4.5 with extended reasoning',
        provider: 'openrouter',
        contextLength: 200000,
        enabled: true,
        isCustom: false,
      },
      {
        id: 'anthropic/claude-sonnet-4',
        name: '✨ Sonnet 4',
        description: 'Anthropic Sonnet 4 model',
        provider: 'openrouter',
        contextLength: 200000,
        enabled: true,
        isCustom: false,
      },
      {
        id: 'anthropic/claude-sonnet-4-thinking',
        name: '✨ Sonnet 4 Thinking',
        description: 'Sonnet 4 with extended reasoning',
        provider: 'openrouter',
        contextLength: 200000,
        enabled: true,
        isCustom: false,
      },
      {
        id: 'anthropic/claude-opus-4',
        name: '✨ Opus 4',
        description: 'Anthropic\'s most powerful model',
        provider: 'openrouter',
        contextLength: 200000,
        enabled: true,
        isCustom: false,
      },
      {
        id: 'anthropic/claude-opus-4-thinking',
        name: '✨ Opus 4 Thinking',
        description: 'Opus 4 with extended reasoning',
        provider: 'openrouter',
        contextLength: 200000,
        enabled: true,
        isCustom: false,
      },
      {
        id: 'anthropic/claude-opus-4.5',
        name: '✨ Opus 4.5',
        description: 'Enhanced Opus 4 model',
        provider: 'openrouter',
        contextLength: 200000,
        enabled: true,
        isCustom: false,
      },
      {
        id: 'anthropic/claude-opus-4.5-thinking',
        name: '✨ Opus 4.5 Thinking',
        description: 'Opus 4.5 with extended reasoning',
        provider: 'openrouter',
        contextLength: 200000,
        enabled: true,
        isCustom: false,
      },
      {
        id: 'anthropic/claude-opus-4.5-high-effort',
        name: '✨ Opus 4.5 High Effort Thinking',
        description: 'Opus 4.5 with maximum reasoning effort',
        provider: 'openrouter',
        contextLength: 200000,
        enabled: true,
        isCustom: false,
      },
      {
        id: 'anthropic/claude-haiku-4.5',
        name: '✨ Haiku 4.5',
        description: 'Fast and efficient Haiku model',
        provider: 'openrouter',
        contextLength: 200000,
        enabled: true,
        isCustom: false,
      },
      {
        id: 'anthropic/claude-haiku-4.5-thinking',
        name: '✨ Haiku 4.5 Thinking',
        description: 'Haiku 4.5 with extended reasoning',
        provider: 'openrouter',
        contextLength: 200000,
        enabled: true,
        isCustom: false,
      },
      // Other Models
      {
        id: 'deepagent/deepagent',
        name: '🤖 DeepAgent',
        description: 'Advanced AI agent model',
        provider: 'openrouter',
        contextLength: 128000,
        enabled: true,
        isCustom: false,
      },
      // HuggingFace Models (Free, No API Key Required)
      {
        id: 'huggingface:mistralai/Mistral-7B-Instruct-v0.2',
        name: '🤗 Mistral 7B (Free)',
        description: 'Free Mistral 7B model via HuggingFace - No API key needed',
        provider: 'huggingface',
        contextLength: 8192,
        enabled: true,
        isCustom: false,
      },
      {
        id: 'huggingface:meta-llama/Llama-2-7b-chat-hf',
        name: '🤗 Llama 7B (Free)',
        description: 'Free Llama 7B Chat model via HuggingFace - No API key needed',
        provider: 'huggingface',
        contextLength: 4096,
        enabled: true,
        isCustom: false,
      },
      {
        id: 'huggingface:codellama/CodeLlama-7b-Instruct-hf',
        name: '🤗 CodeLlama 7B (Free)',
        description: 'Free CodeLlama 7B Instruct model via HuggingFace - No API key needed',
        provider: 'huggingface',
        contextLength: 16384,
        enabled: true,
        isCustom: false,
      },
      {
        id: 'qwen/qwen3-coder',
        name: '💜 Qwen3 Coder',
        description: 'Alibaba\'s coding model',
        provider: 'openrouter',
        contextLength: 128000,
        enabled: true,
        isCustom: false,
      },
      {
        id: 'x-ai/grok-code-fast',
        name: '⚫ Grok Code Fast',
        description: 'X.AI\'s fast coding model',
        provider: 'openrouter',
        contextLength: 128000,
        enabled: true,
        isCustom: false,
      },
      {
        id: 'x-ai/grok-4',
        name: '⚫ Grok 4',
        description: 'X.AI\'s latest model',
        provider: 'openrouter',
        contextLength: 128000,
        enabled: true,
        isCustom: false,
      },
      {
        id: 'zai/glm-4.6',
        name: '⚫ ZAI GLM 4.6',
        description: 'Zhipu AI\'s GLM model',
        provider: 'openrouter',
        contextLength: 128000,
        enabled: true,
        isCustom: false,
      },
      {
        id: 'kimi/kimi-k2-turbo',
        name: '⚫ Kimi K2 Turbo',
        description: 'Moonshot AI\'s fast model',
        provider: 'openrouter',
        contextLength: 128000,
        enabled: true,
        isCustom: false,
      },
      // MCP-enabled models (require service connections)
      {
        id: 'mcp-github-enhanced',
        name: '🤖 GitHub-Enhanced AI',
        description: 'AI with GitHub context - Access repos, issues, PRs',
        provider: 'mcp',
        contextLength: 200000,
        enabled: false,
        isCustom: false,
        apiEndpoint: 'mcp://github',
      },
      {
        id: 'mcp-slack-enhanced',
        name: '💬 Slack-Enhanced AI',
        description: 'AI with Slack context - Team communication insights',
        provider: 'mcp',
        contextLength: 200000,
        enabled: false,
        isCustom: false,
        apiEndpoint: 'mcp://slack',
      },
      {
        id: 'mcp-linear-enhanced',
        name: '📋 Linear-Enhanced AI',
        description: 'AI with Linear context - Project management insights',
        provider: 'mcp',
        contextLength: 200000,
        enabled: false,
        isCustom: false,
        apiEndpoint: 'mcp://linear',
      },
    ];

    return {
      models: defaultModels,
      providers: {
        gemini: { provider: 'gemini', apiKey: '', enabled: true },
        openrouter: { provider: 'openrouter', apiKey: '', enabled: true },
        openai: { provider: 'openai', apiKey: '', enabled: false },
        anthropic: { provider: 'anthropic', apiKey: '', enabled: false },
        custom: { provider: 'custom', apiKey: '', enabled: false },
        mcp: { provider: 'mcp', apiKey: '', enabled: true },
      },
      defaultModel: 'gemini-1.5-pro-latest',
    };
  }

  private saveConfig(config: {
    models: AIModel[];
    providers: Record<ModelProvider, ProviderConfig>;
    defaultModel: string;
  }): void {
    // Don't save API keys in config - they're stored separately
    const configToSave = {
      ...config,
      providers: Object.fromEntries(
        Object.entries(config.providers).map(([key, value]) => [
          key,
          { ...value, apiKey: '' }, // Remove API key from saved config
        ])
      ),
    };
    localStorage.setItem(this.STORAGE_KEY, JSON.stringify(configToSave));
  }

  private getApiKeys(): Record<ModelProvider, string> {
    const stored = localStorage.getItem(this.API_KEYS_STORAGE_KEY);
    if (stored) {
      try {
        return JSON.parse(stored);
      } catch {
        // Fall through to defaults
      }
    }
    return {
      gemini: import.meta.env.VITE_GEMINI_API_KEY || '',
      openrouter: import.meta.env.VITE_OPENROUTER_API_KEY || '',
      openai: '',
      anthropic: '',
      custom: '',
    };
  }

  private saveApiKeys(keys: Record<ModelProvider, string>): void {
    localStorage.setItem(this.API_KEYS_STORAGE_KEY, JSON.stringify(keys));
  }
}

export const modelConfigService = new ModelConfigService();

