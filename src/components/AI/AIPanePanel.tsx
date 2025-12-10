import React, { useState, useRef, useEffect } from 'react';
import './AIPanePanel.css';
import { mcpCodeGenerator } from '../../services/mcp-code-generator';
import { projectScaffolder } from '../../services/project-scaffolder';
import { Mic, MicOff, Volume2, VolumeX, Settings as SettingsIcon, Paperclip, X, File, Image as ImageIcon, FileText, Video, FileCode, Globe, Browser } from 'lucide-react';
import { voiceService } from '../../services/voice-service';
import { modelConfigService, type AIModel } from '../../services/model-config-service';
import { WelcomeScreen } from './WelcomeScreen';
import { authService } from '../../services/auth-service';
import { aiCodeGeneratorService } from '../../services/ai-code-generator-service';
import { freeLLMService } from '../../services/free-llm-service';
import { navaAssistantService } from '../../services/nava-assistant-service';
import { navaAgentsService } from '../../services/nava-agents-service';
import { navaToolsService } from '../../services/nava-tools-service';
import { backendRegistry, costTracker } from '../../services/llm-hub';
import { getBackendIdFromModel, initializeBackends } from '../../services/llm-hub/backend-initializer';
import { navaAITools } from '../../services/nava-ai-tools';
import { navaRuntimeService } from '../../services/nava-runtime-service';
import { navaPreviewEngine } from '../../services/nava-preview-engine';

interface UploadedFile {
  id: string;
  name: string;
  size: number;
  type: string;
  content?: string | ArrayBuffer;
  preview?: string;
}

interface Message {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  timestamp: Date;
  files?: UploadedFile[];
}

interface AIModel {
  id: string;
  name: string;
  description: string;
  contextLength: number;
}

interface AIModelWithProvider extends AIModel {
  provider: 'gemini' | 'openrouter' | 'custom' | 'nava-local';
  apiEndpoint?: string;
}

interface AIPanePanelProps {
  currentCode?: string;
  currentFile?: string;
}

export const AIPanePanel: React.FC<AIPanePanelProps> = ({ 
  currentCode: propCurrentCode,
  currentFile: propCurrentFile,
}) => {
  // Load models from configuration service (includes custom models)
  const [availableModels, setAvailableModels] = useState<AIModelWithProvider[]>(() => {
    // Get all models and check if we need to refresh
    const allModels = modelConfigService.getModels();
    const enabledModels = allModels.filter(m => m.enabled);
    
    // If we have very few models, refresh from defaults
    if (enabledModels.length < 10) {
      console.log('[AI Panel] Only', enabledModels.length, 'models found. Refreshing...');
      modelConfigService.refreshModels();
      // Get fresh list after refresh
      const freshModels = modelConfigService.getModels().filter(m => m.enabled);
      return freshModels.map(model => ({
        id: model.id,
        name: model.name,
        description: model.description,
        contextLength: model.contextLength,
        provider: model.provider as 'gemini' | 'openrouter',
        apiEndpoint: model.isCustom ? model.apiEndpoint : undefined,
      }));
    }
    
    return enabledModels.map(model => ({
      id: model.id,
      name: model.name,
      description: model.description,
      contextLength: model.contextLength,
      provider: model.provider as 'gemini' | 'openrouter',
      apiEndpoint: model.isCustom ? model.apiEndpoint : undefined,
    }));
  });

  // Initialize backends on mount and when API keys change
  useEffect(() => {
    console.log('[AI Panel] Initializing backends on mount...');
    initializeBackends();
    console.log('[AI Panel] Available backends:', backendRegistry.getAvailableBackends().map(b => b.id));
  }, []);

  // Refresh models when settings change
  useEffect(() => {
    const refreshModels = () => {
      // Get all models (not just enabled) to show full list
      const allModels = modelConfigService.getModels();
      // Filter to only show enabled models
      const enabledModels = allModels.filter(m => m.enabled);
      
      // If we have fewer than 10 models, something might be wrong - refresh from defaults
      if (enabledModels.length < 10) {
        console.warn('[AI Panel] Only', enabledModels.length, 'models found. Refreshing model list...');
        modelConfigService.refreshModels();
        // Get fresh list
        const freshModels = modelConfigService.getModels().filter(m => m.enabled);
        setAvailableModels(freshModels.map(model => ({
          id: model.id,
          name: model.name,
          description: model.description,
          contextLength: model.contextLength,
          provider: model.provider as 'gemini' | 'openrouter',
          apiEndpoint: model.isCustom ? model.apiEndpoint : undefined,
        })));
      } else {
        setAvailableModels(enabledModels.map(model => ({
          id: model.id,
          name: model.name,
          description: model.description,
          contextLength: model.contextLength,
          provider: model.provider as 'gemini' | 'openrouter',
          apiEndpoint: model.isCustom ? model.apiEndpoint : undefined,
        })));
      }
    };

    // Initial load
    refreshModels();

    // Refresh on storage change (when settings are updated)
    window.addEventListener('storage', refreshModels);
    // Also check periodically (for same-window updates)
    const interval = setInterval(refreshModels, 2000);

    return () => {
      window.removeEventListener('storage', refreshModels);
      clearInterval(interval);
    };
  }, []);

  // Fallback models if none configured
  const fallbackModels: AIModelWithProvider[] = [
    // Google Gemini Models (Direct API - Default)
    {
      id: 'gemini-2.0-flash-exp',
      name: '🚀 Gemini 2.0 Flash (Experimental)',
      description: 'Latest Gemini - Fast and powerful (FREE with Google account)',
      contextLength: 1000000,
      provider: 'gemini',
    },
    {
      id: 'gemini-1.5-pro-latest',
      name: '⭐ Gemini 1.5 Pro (Latest)',
      description: 'Most capable Gemini - 1M context window (FREE)',
      contextLength: 1000000,
      provider: 'gemini',
    },
    {
      id: 'gemini-1.5-flash-latest',
      name: '⚡ Gemini 1.5 Flash (Latest)',
      description: 'Fast multimodal model - 1M context (FREE)',
      contextLength: 1000000,
      provider: 'gemini',
    },
    {
      id: 'gemini-1.5-pro',
      name: 'Gemini 1.5 Pro',
      description: 'Google\'s most capable model - 1M context',
      contextLength: 1000000,
      provider: 'gemini',
    },
    {
      id: 'gemini-1.5-flash',
      name: 'Gemini 1.5 Flash',
      description: 'Fast multimodal model',
      contextLength: 1000000,
      provider: 'gemini',
    },
    {
      id: 'gemini-pro',
      name: 'Gemini Pro',
      description: 'Standard Gemini model',
      contextLength: 32000,
      provider: 'gemini',
    },
    
    // Anthropic Claude Models (OpenRouter)
    {
      id: 'anthropic/claude-3.5-sonnet',
      name: '✨ Claude 3.5 Sonnet',
      description: '⭐ BEST for coding! Most capable model for code generation',
      contextLength: 200000,
      provider: 'openrouter',
    },
    {
      id: 'anthropic/claude-3-opus',
      name: 'Claude 3 Opus',
      description: 'Most powerful - Best for complex reasoning',
      contextLength: 200000,
      provider: 'openrouter',
    },
    {
      id: 'anthropic/claude-3.5-haiku',
      name: 'Claude 3.5 Haiku',
      description: 'Latest fast Claude - Great for simple tasks',
      contextLength: 200000,
      provider: 'openrouter',
    },
    {
      id: 'anthropic/claude-3-sonnet',
      name: 'Claude 3 Sonnet',
      description: 'Balanced performance and speed',
      contextLength: 200000,
      provider: 'openrouter',
    },
    {
      id: 'anthropic/claude-3-haiku',
      name: 'Claude 3 Haiku',
      description: 'Fastest Claude model - Great for simple tasks',
      contextLength: 200000,
      provider: 'openrouter',
    },
    
    // OpenAI Models (Latest - OpenRouter)
    {
      id: 'openai/gpt-4o',
      name: '🎯 GPT-4o (Latest)',
      description: 'Latest GPT-4 Omni - Multimodal, 128K context',
      contextLength: 128000,
      provider: 'openrouter',
    },
    {
      id: 'openai/gpt-4o-mini',
      name: 'GPT-4o Mini',
      description: 'Fast and efficient GPT-4o variant',
      contextLength: 128000,
      provider: 'openrouter',
    },
    {
      id: 'openai/gpt-4-turbo',
      name: '🚀 GPT-4 Turbo',
      description: 'GPT-4 Turbo with 128K context - Vision capable',
      contextLength: 128000,
      provider: 'openrouter',
    },
    {
      id: 'openai/gpt-4-turbo-preview',
      name: 'GPT-4 Turbo Preview',
      description: 'Preview version of GPT-4 Turbo',
      contextLength: 128000,
      provider: 'openrouter',
    },
    {
      id: 'openai/gpt-4',
      name: 'GPT-4',
      description: 'OpenAI flagship model - Most capable GPT',
      contextLength: 8192,
      provider: 'openrouter',
    },
    {
      id: 'openai/gpt-3.5-turbo',
      name: 'GPT-3.5 Turbo',
      description: 'Fast and cost-effective',
      contextLength: 16385,
      provider: 'openrouter',
    },
    {
      id: 'openai/o1-preview',
      name: '🧠 OpenAI O1 Preview',
      description: 'Advanced reasoning model - Best for complex problems',
      contextLength: 128000,
      provider: 'openrouter',
    },
    {
      id: 'openai/o1-mini',
      name: 'OpenAI O1 Mini',
      description: 'Faster reasoning model',
      contextLength: 128000,
      provider: 'openrouter',
    },
    
    // Google Models (OpenRouter - for comparison)
    {
      id: 'google/gemini-pro',
      name: 'Gemini Pro (OpenRouter)',
      description: 'Gemini via OpenRouter',
      contextLength: 32000,
      provider: 'openrouter',
    },
    {
      id: 'google/gemini-1.5-pro',
      name: 'Gemini 1.5 Pro (OpenRouter)',
      description: 'Gemini 1.5 Pro via OpenRouter',
      contextLength: 1000000,
      provider: 'openrouter',
    },
    
    // Meta Llama Models (OpenRouter)
    {
      id: 'meta-llama/llama-3.1-405b-instruct',
      name: '🦙 Llama 3.1 405B',
      description: 'Largest open model - Extremely powerful!',
      contextLength: 128000,
      provider: 'openrouter',
    },
    {
      id: 'meta-llama/llama-3.1-70b-instruct',
      name: 'Llama 3.1 70B',
      description: 'High performance open model',
      contextLength: 128000,
      provider: 'openrouter',
    },
    {
      id: 'meta-llama/llama-3.1-8b-instruct',
      name: 'Llama 3.1 8B',
      description: 'Fast and efficient',
      contextLength: 128000,
      provider: 'openrouter',
    },
    {
      id: 'meta-llama/llama-3-70b-instruct',
      name: 'Llama 3 70B',
      description: 'Previous gen - Still powerful',
      contextLength: 8192,
      provider: 'openrouter',
    },
    
    // Mistral Models (OpenRouter)
    {
      id: 'mistralai/mistral-large',
      name: '🇪🇺 Mistral Large',
      description: 'Top European model - Excellent for code',
      contextLength: 32000,
      provider: 'openrouter',
    },
    {
      id: 'mistralai/mixtral-8x7b-instruct',
      name: 'Mixtral 8x7B',
      description: 'MoE model - Fast and powerful',
      contextLength: 32000,
      provider: 'openrouter',
    },
    {
      id: 'mistralai/mixtral-8x22b',
      name: 'Mixtral 8x22B',
      description: 'Largest Mixtral - 141B params total',
      contextLength: 64000,
      provider: 'openrouter',
    },
    
    // Specialized Models (OpenRouter)
    {
      id: 'deepseek/deepseek-coder-33b-instruct',
      name: '💻 DeepSeek Coder 33B',
      description: 'Specialized for code generation',
      contextLength: 16000,
      provider: 'openrouter',
    },
    {
      id: 'cohere/command-r-plus',
      name: 'Command R+',
      description: 'Enterprise-grade model with RAG',
      contextLength: 128000,
      provider: 'openrouter',
    },
    {
      id: 'perplexity/llama-3.1-sonar-large-128k-online',
      name: '🔍 Perplexity Sonar',
      description: 'Online search-enhanced model',
      contextLength: 128000,
      provider: 'openrouter',
    },
  ];

  // Use configured models or fallback
  // Sort models alphabetically by name (case-insensitive, ignoring emojis)
  const sortModelsAlphabetically = (models: AIModelWithProvider[]): AIModelWithProvider[] => {
    return [...models].sort((a, b) => {
      // Remove emojis and special characters for sorting
      const nameA = a.name.replace(/[^\w\s]/g, '').toLowerCase();
      const nameB = b.name.replace(/[^\w\s]/g, '').toLowerCase();
      return nameA.localeCompare(nameB);
    });
  };

  const modelsToUse = sortModelsAlphabetically(
    availableModels.length > 0 ? availableModels : fallbackModels
  );

  const [showWelcome, setShowWelcome] = useState(() => {
    // Show welcome screen if no messages exist
    const hasMessages = localStorage.getItem('nava_ai_has_messages') === 'true';
    return !hasMessages;
  });

  const [messages, setMessages] = useState<Message[]>(() => {
    // Check if user has API key configured (check directly from sources)
    const envGeminiKey = import.meta.env.VITE_GEMINI_API_KEY;
    const envOpenRouterKey = import.meta.env.VITE_OPENROUTER_API_KEY;
    const localGeminiKey = localStorage.getItem('gemini_api_key');
    const localOpenRouterKey = localStorage.getItem('openrouter_api_key');
    const hasApiKey = !!(envGeminiKey || envOpenRouterKey || localGeminiKey || localOpenRouterKey);
    
    const initialMessage = hasApiKey
      ? '👋 Hello! I\'m your NAVΛ AI Assistant, powered by AI. I can help you with:\n\n• Writing and debugging code in any language\n• Generating code from descriptions\n• Explaining code and concepts\n• Refactoring and optimizing\n• Answering technical questions\n\nAsk me anything - I\'ll respond conversationally, just like Cursor!'
      : '👋 Hello! I\'m your NAVΛ AI Assistant.\n\nTo get full conversational AI assistance (like Cursor), please add your API key:\n\n1. Click the ⚙️ icon above\n2. Add your Gemini API key (FREE from https://ai.google.dev/)\n   Or add an OpenRouter API key\n\nOnce configured, I can provide real-time, contextual AI responses!';
    
    return [{
      id: '1',
      role: 'assistant',
      content: initialMessage,
      timestamp: new Date(),
    }];
  });

  const [input, setInput] = useState('');
  const [isThinking, setIsThinking] = useState(false);
  // Default to Gemini 1.5 Pro (first Gemini model)
  const [selectedModel, setSelectedModel] = useState<string>(() => {
    const saved = localStorage.getItem('selected_ai_model');
    return saved || 'gemini-1.5-pro-latest';
  });
  // Load API keys from environment variables first, then modelConfigService, then localStorage
  const [geminiApiKey, setGeminiApiKey] = useState<string>(() => {
    // Check environment variable first (Vite uses import.meta.env)
    const envKey = import.meta.env.VITE_GEMINI_API_KEY;
    if (envKey) {
      return envKey;
    }
    // Check modelConfigService
    const serviceKey = modelConfigService.getProviderApiKey('gemini');
    if (serviceKey) {
      return serviceKey;
    }
    // Fallback to localStorage
    return localStorage.getItem('gemini_api_key') || '';
  });
  const [openRouterApiKey, setOpenRouterApiKey] = useState<string>(() => {
    // Check environment variable first
    const envKey = import.meta.env.VITE_OPENROUTER_API_KEY;
    if (envKey) {
      return envKey;
    }
    // Check modelConfigService
    const serviceKey = modelConfigService.getProviderApiKey('openrouter');
    if (serviceKey) {
      return serviceKey;
    }
    // Fallback to localStorage
    return localStorage.getItem('openrouter_api_key') || '';
  });
  
  // Sync API keys with modelConfigService when they change and re-initialize backends
  useEffect(() => {
    if (geminiApiKey) {
      modelConfigService.setProviderApiKey('gemini', geminiApiKey);
      // Re-initialize backends when API key changes
      console.log('[AI Panel] Gemini API key updated, re-initializing backends...');
      initializeBackends();
    }
  }, [geminiApiKey]);
  
  useEffect(() => {
    if (openRouterApiKey) {
      modelConfigService.setProviderApiKey('openrouter', openRouterApiKey);
      // Re-initialize backends when API key changes
      console.log('[AI Panel] OpenRouter API key updated, re-initializing backends...');
      initializeBackends();
    }
  }, [openRouterApiKey]);
  const [showSettings, setShowSettings] = useState(false);
  
  // Voice control states
  const [isVoiceEnabled, setIsVoiceEnabled] = useState(false);
  const [isListening, setIsListening] = useState(false);
  const [showVoiceSettings, setShowVoiceSettings] = useState(false);
  
  // File upload state
  const [uploadedFiles, setUploadedFiles] = useState<UploadedFile[]>([]);
  
  // Refs
  const fileInputRef = useRef<HTMLInputElement>(null);
  const messagesEndRef = useRef<HTMLDivElement>(null);

  const scrollToBottom = () => {
    if (messagesEndRef.current) {
      messagesEndRef.current.scrollIntoView({ behavior: 'smooth', block: 'end' });
    }
  };
  
  // Auto-scroll when messages change or when thinking indicator appears
  useEffect(() => {
    // Use setTimeout to ensure DOM is updated before scrolling
    const timer = setTimeout(() => {
      scrollToBottom();
    }, 100);
    return () => clearTimeout(timer);
  }, [messages, isThinking]);

  // Auto-scroll when messages change or when thinking indicator appears
  useEffect(() => {
    // Use setTimeout to ensure DOM is updated
    const timer = setTimeout(() => {
      scrollToBottom();
    }, 100);
    return () => clearTimeout(timer);
  }, [messages, isThinking]);

  // Detect programming language from prompt
  const detectLanguage = (prompt: string): string => {
    const lower = prompt.toLowerCase();
    if (lower.includes('python') || lower.includes('py')) return 'python';
    if (lower.includes('javascript') || lower.includes('js') || lower.includes('typescript') || lower.includes('ts')) return 'typescript';
    if (lower.includes('rust')) return 'rust';
    if (lower.includes('sql')) return 'sql';
    if (lower.includes('r ')) return 'r';
    if (lower.includes('nava') || lower.includes('⋋') || lower.includes('vnc')) return 'navlambda';
    return 'typescript'; // Default
  };

  // Detect framework from prompt
  const detectFramework = (prompt: string): string => {
    const lower = prompt.toLowerCase();
    if (lower.includes('react')) return 'react';
    if (lower.includes('vue')) return 'vue';
    if (lower.includes('angular')) return 'angular';
    if (lower.includes('next')) return 'nextjs';
    if (lower.includes('express')) return 'express';
    if (lower.includes('fastapi') || lower.includes('flask')) return 'python';
    return 'none';
  };

  // Handle code generation and insertion into editor
  const handleCodeGeneration = async (aiContent: string, userPrompt: string) => {
    // Detect if user wants code generation
    const lowerPrompt = userPrompt.toLowerCase();
    const isCodeRequest = lowerPrompt.includes('create') || 
                         lowerPrompt.includes('generate') || 
                         lowerPrompt.includes('write') || 
                         lowerPrompt.includes('build') || 
                         lowerPrompt.includes('implement') ||
                         lowerPrompt.includes('add') ||
                         lowerPrompt.includes('make') ||
                         aiContent.includes('```');

    if (isCodeRequest) {
      try {
        // Use AI code generator service
        const result = await aiCodeGeneratorService.generateCode({
          prompt: userPrompt,
          context: {
            currentCode: '', // Could get from editor
            language: detectLanguage(userPrompt),
            framework: detectFramework(userPrompt),
          },
        });

        // Emit event to insert code into editor
        const event = new CustomEvent('nava:insert-code', {
          detail: {
            code: result.code,
            language: result.language,
            explanation: result.explanation,
            files: result.files,
          },
        });
        window.dispatchEvent(event);
      } catch (error) {
        console.error('Code generation error:', error);
      }
    }
  };

  const handleSend = async () => {
    if (!input.trim() || isThinking) return;

    const userMessage: Message = {
      id: Date.now().toString(),
      role: 'user',
      content: input,
      timestamp: new Date(),
      files: uploadedFiles.length > 0 ? [...uploadedFiles] : undefined,
    };

    setMessages(prev => [...prev, userMessage]);
    const query = input;
    setInput('');
    setUploadedFiles([]); // Clear uploaded files after sending
    setIsThinking(true);

    try {
      // ALWAYS use the selected model through LLM Hub (real models, not predefined responses)
      const selectedModelInfo = modelsToUse.find(m => m.id === selectedModel);
      const provider = selectedModelInfo?.provider || 'gemini';
      
      // Get backend ID from selected model
      const backendId = getBackendIdFromModel(selectedModel, provider);
      console.log(`[AI Panel] Model: ${selectedModel}, Provider: ${provider}, Backend ID: ${backendId}`);
      
      // Get backend from registry
      let backend = backendRegistry.getBackend(backendId);
      
      // If backend not found, try to re-initialize and check again
      if (!backend) {
        console.warn(`[AI Panel] Backend ${backendId} not found. Re-initializing backends...`);
        initializeBackends();
        backend = backendRegistry.getBackend(backendId);
      }
      
      // If still not found, try default backend
      if (!backend) {
        console.warn(`[AI Panel] Backend ${backendId} still not found after re-initialization. Trying default backend...`);
        backend = backendRegistry.getDefaultBackend();
        if (!backend) {
          // Try to get HuggingFace backend as fallback (always available)
          const huggingfaceBackend = backendRegistry.getBackend('huggingface:mistralai/Mistral-7B-Instruct-v0.2');
          if (huggingfaceBackend) {
            console.warn(`[AI Panel] Using HuggingFace fallback backend`);
            backend = huggingfaceBackend;
          } else {
            const availableBackends = backendRegistry.getAvailableBackends().map(b => b.id);
            throw new Error(
              `Model "${selectedModel}" is not available.\n\n` +
              `**Backend ID:** ${backendId}\n` +
              `**Available backends:** ${availableBackends.join(', ')}\n\n` +
              `**To use this model:**\n` +
              `- Claude 3.5 Sonnet requires OpenRouter API key (Settings → ⚙️ icon)\n` +
              `- Make sure API keys are saved and backends are initialized\n` +
              `- Or select a HuggingFace model (free, no API key needed)`
            );
          }
        } else {
          console.warn(`[AI Panel] Using default backend: ${backend.id}`);
        }
      }
      
      // Get current code context
      const currentCode = propCurrentCode || '';
      const currentFilePath = propCurrentFile || '';
      
      // Build context for NAVA system prompt
      const context = {
        currentFile: currentFilePath,
        currentCode: currentCode,
        language: currentFilePath ? fileService.getFileExtension(currentFilePath) : undefined,
      };
      
      // Use backend registry chat which includes NAVA system prompt and tools
      console.log(`[AI Panel] Using backend: ${backend.id} for model: ${selectedModel}`);
      console.log(`[AI Panel] Backend available: ${backend.isAvailable()}`);
      console.log(`[AI Panel] All available backends:`, backendRegistry.getAvailableBackends().map(b => b.id));
      
      // Check if backend is actually available
      if (!backend.isAvailable()) {
        console.warn(`[AI Panel] Backend ${backend.id} is not available, trying fallback...`);
        // Try HuggingFace as fallback
        const fallbackBackend = backendRegistry.getBackend('huggingface:mistralai/Mistral-7B-Instruct-v0.2') || 
                                backendRegistry.getDefaultBackend();
        if (fallbackBackend && fallbackBackend.isAvailable()) {
          console.log(`[AI Panel] Using fallback backend: ${fallbackBackend.id}`);
          backend = fallbackBackend;
        } else {
          throw new Error(
            `Selected model "${selectedModel}" is not available.\n\n` +
            `**Available options:**\n` +
            `1. Add API key in Settings (⚙️ icon) for ${selectedModel}\n` +
            `2. Select a HuggingFace model (free, no API key)\n` +
            `3. Start NAVA Local server if using NAVA Local model`
          );
        }
      }
      
      const response = await backendRegistry.chat(
        query,
        context,
        backend.id,
        navaAITools.getAvailableTools()
      );
      
      let aiContent = response.text || 'No response received.';
      
      // Handle tool calls if present
      if (response.toolCalls && response.toolCalls.length > 0) {
        console.log(`[AI Panel] Executing ${response.toolCalls.length} tool calls...`);
        
        // Execute tools and get results
        const toolResults: Array<{ tool_call_id: string; content: string }> = [];
        
        for (const toolCall of response.toolCalls) {
          try {
            const toolArgs = JSON.parse(toolCall.function.arguments);
            const toolResult = await navaAITools.executeTool(toolCall.function.name, toolArgs);
            
            toolResults.push({
              tool_call_id: toolCall.id,
              content: JSON.stringify(toolResult),
            });
          } catch (toolError) {
            console.error('[Tool Execution] Error:', toolError);
            toolResults.push({
              tool_call_id: toolCall.id,
              content: JSON.stringify({ success: false, error: String(toolError) }),
            });
          }
        }
        
        // Send tool results back to model for final response
        if (toolResults.length > 0) {
          const toolMessages = [
            ...messages.slice(-10).map(m => ({
              role: m.role as 'user' | 'assistant' | 'system',
              content: m.content,
            })),
            {
              role: 'user' as const,
              content: query,
            },
            {
              role: 'assistant' as const,
              content: aiContent,
              tool_calls: response.toolCalls,
            },
            ...toolResults.map(tr => ({
              role: 'tool' as const,
              content: tr.content,
              tool_call_id: tr.tool_call_id,
            })),
          ];
          
          const toolResponse = await backend.chat({
            messages: toolMessages,
            temperature: 0.2,
            maxTokens: 2000,
          });
          
          aiContent = toolResponse.text || aiContent;
        }
      }
      
      // Check if this is a code generation request
      await handleCodeGeneration(aiContent, query);
      
      // Extract and insert code if present
      const codeMatch = aiContent.match(/```(?:nava|navlambda|vnc)?\n([\s\S]*?)```/);
      if (codeMatch) {
        const navaCode = codeMatch[1];
        
        // Emit code insertion event
        const event = new CustomEvent('nava:insert-code', {
          detail: {
            code: navaCode,
            language: 'navlambda',
            explanation: aiContent.replace(/```[\s\S]*?```/g, '').trim(),
          },
        });
        window.dispatchEvent(event);
      }
      
      const aiResponse: Message = {
        id: (Date.now() + 1).toString(),
        role: 'assistant',
        content: aiContent,
        timestamp: new Date(),
      };
      
      setMessages(prev => [...prev, aiResponse]);
      setIsThinking(false);
      return;
    } catch (error: any) {
      console.error('[AI Panel] Error:', error);
      
      // Show helpful error message
      const errorMessage: Message = {
        id: (Date.now() + 1).toString(),
        role: 'assistant',
        content: `⚠️ **Error:** ${error.message || 'Failed to get AI response'}\n\n**Troubleshooting:**\n1. Check your API keys in Settings (⚙️ icon)\n2. For NAVA Local: Ensure model server is running (` + 
                 `\`./scripts/start_nava_model.sh\`)\n3. Check browser console for details\n4. Try selecting a different model`,
        timestamp: new Date(),
      };
      setMessages(prev => [...prev, errorMessage]);
    }

    setIsThinking(false);
  };

  // This function is no longer used - we use real AI responses instead
  // Keeping for backward compatibility but it should not be called
  const generateAIResponse = (query: string): string => {
    return `I'd love to help you with that! To get full conversational AI assistance, please add your API key in settings. Once configured, I can provide real-time, contextual responses just like Cursor!`;
  };

  const handleKeyDown = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSend();
    }
  };

  // NAVA-specific symbol insertions with proper syntax
  const insertNAVASymbol = (symbol: string, navaSyntax?: string) => {
    // Insert symbol at cursor position or append to end
    const textarea = document.querySelector('.prompt-textarea') as HTMLTextAreaElement;
    if (textarea) {
      const start = textarea.selectionStart;
      const end = textarea.selectionEnd;
      const text = input;
      // Use NAVA syntax if provided, otherwise use symbol
      const insertText = navaSyntax || symbol;
      const newText = text.substring(0, start) + insertText + text.substring(end);
      setInput(newText);
      // Restore cursor position after symbol
      setTimeout(() => {
        textarea.focus();
        textarea.setSelectionRange(start + insertText.length, start + insertText.length);
      }, 0);
    } else {
      // Fallback: append to end
      setInput(prev => prev + (navaSyntax || symbol));
    }
  };

  // File upload handlers
  const handleFileUpload = async (event: React.ChangeEvent<HTMLInputElement>) => {
    const files = event.target.files;
    if (!files) return;

    const newFiles: UploadedFile[] = [];

    for (let i = 0; i < files.length; i++) {
      const file = files[i];
      const fileData: UploadedFile = {
        id: `${Date.now()}-${i}`,
        name: file.name,
        size: file.size,
        type: file.type || 'application/octet-stream',
      };

      // Read file content based on type
      const fileExtension = file.name.toLowerCase().split('.').pop() || '';
      const isTextFile = file.type.startsWith('text/') || 
                        file.type.includes('json') || 
                        file.type.includes('javascript') ||
                        file.type.includes('typescript') ||
                        file.type.includes('xml') ||
                        ['csv', 'txt', 'md', 'py', 'js', 'ts', 'jsx', 'tsx', 'json', 'xml', 'html', 'css', 'scss', 'sass', 'less', 'yaml', 'yml', 'toml', 'ini', 'conf', 'config', 'log', 'sh', 'bash', 'zsh', 'fish', 'ps1', 'bat', 'cmd', 'ipynb', 'r', 'R', 'sql', 'pl', 'pm', 'rb', 'go', 'rs', 'java', 'kt', 'scala', 'clj', 'hs', 'elm', 'ex', 'exs', 'ml', 'mli', 'fs', 'fsx', 'vb', 'cs', 'cpp', 'cxx', 'cc', 'c', 'h', 'hpp', 'hxx', 'swift', 'm', 'mm', 'dart', 'lua', 'vim', 'vimrc'].includes(fileExtension);
      
      if (file.type.startsWith('image/') || ['jpg', 'jpeg', 'png', 'gif', 'webp', 'svg', 'bmp', 'ico', 'tiff', 'tif'].includes(fileExtension)) {
        // Images - read as data URL for preview
        const reader = new FileReader();
        reader.onload = (e) => {
          fileData.preview = e.target?.result as string;
          fileData.content = e.target?.result;
        };
        reader.readAsDataURL(file);
      } else if (file.type.startsWith('video/') || ['mp4', 'webm', 'ogg', 'mov', 'avi', 'mkv', 'flv', 'wmv', 'm4v'].includes(fileExtension)) {
        // Videos - read as data URL
        const reader = new FileReader();
        reader.onload = (e) => {
          fileData.preview = e.target?.result as string;
          fileData.content = e.target?.result;
        };
        reader.readAsDataURL(file);
      } else if (file.type.startsWith('audio/') || ['mp3', 'wav', 'ogg', 'flac', 'aac', 'm4a', 'wma', 'opus'].includes(fileExtension)) {
        // Audio - read as data URL
        const reader = new FileReader();
        reader.onload = (e) => {
          fileData.content = e.target?.result;
        };
        reader.readAsDataURL(file);
      } else if (isTextFile) {
        // Text-based files - read as text
        const reader = new FileReader();
        reader.onload = (e) => {
          fileData.content = e.target?.result as string;
        };
        reader.readAsText(file, 'utf-8');
      } else if (['pdf', 'doc', 'docx', 'xls', 'xlsx', 'ppt', 'pptx', 'odt', 'ods', 'odp'].includes(fileExtension)) {
        // Office documents - read as data URL (base64)
        const reader = new FileReader();
        reader.onload = (e) => {
          fileData.content = e.target?.result;
        };
        reader.readAsDataURL(file);
      } else if (['zip', 'rar', '7z', 'tar', 'gz', 'bz2', 'xz'].includes(fileExtension)) {
        // Archives - read as data URL
        const reader = new FileReader();
        reader.onload = (e) => {
          fileData.content = e.target?.result;
        };
        reader.readAsDataURL(file);
      } else if (['glb', 'gltf', 'obj', 'fbx', 'dae', '3ds', 'blend'].includes(fileExtension)) {
        // 3D models - read as data URL
        const reader = new FileReader();
        reader.onload = (e) => {
          fileData.content = e.target?.result;
        };
        reader.readAsDataURL(file);
      } else {
        // Default: read as data URL for binary files
        const reader = new FileReader();
        reader.onload = (e) => {
          fileData.content = e.target?.result;
        };
        reader.readAsDataURL(file);
      }

      newFiles.push(fileData);
    }

    setUploadedFiles(prev => [...prev, ...newFiles]);
    
    if (event.target) {
      event.target.value = '';
    }
  };

  const removeFile = (fileId: string) => {
    setUploadedFiles(prev => prev.filter(f => f.id !== fileId));
  };

  const getFileIcon = (file: UploadedFile) => {
    if (file.type.startsWith('image/')) return <ImageIcon size={16} />;
    if (file.type.startsWith('video/')) return <Video size={16} />;
    if (file.type.startsWith('text/') || file.name.endsWith('.txt')) return <FileText size={16} />;
    if (file.type.includes('pdf')) return <FileText size={16} />;
    if (file.type.includes('json') || file.type.includes('javascript') || file.type.includes('typescript')) return <FileCode size={16} />;
    return <File size={16} />;
  };

  const formatFileSize = (bytes: number) => {
    if (bytes < 1024) return bytes + ' B';
    if (bytes < 1024 * 1024) return (bytes / 1024).toFixed(1) + ' KB';
    return (bytes / (1024 * 1024)).toFixed(1) + ' MB';
  };

  const saveGeminiApiKey = (key: string) => {
    setGeminiApiKey(key);
    // Only save to localStorage if not using env variable
    if (!import.meta.env.VITE_GEMINI_API_KEY) {
      localStorage.setItem('gemini_api_key', key);
    } else {
      // Clear localStorage if env var is set (env takes precedence)
      localStorage.removeItem('gemini_api_key');
    }
  };

  const saveOpenRouterApiKey = (key: string) => {
    setOpenRouterApiKey(key);
    // Only save to localStorage if not using env variable
    if (!import.meta.env.VITE_OPENROUTER_API_KEY) {
    localStorage.setItem('openrouter_api_key', key);
    } else {
      // Clear localStorage if env var is set (env takes precedence)
      localStorage.removeItem('openrouter_api_key');
    }
  };

  // Check if keys are loaded from environment
  const isGeminiKeyFromEnv = !!import.meta.env.VITE_GEMINI_API_KEY;
  const isOpenRouterKeyFromEnv = !!import.meta.env.VITE_OPENROUTER_API_KEY;

  useEffect(() => {
    localStorage.setItem('selected_ai_model', selectedModel);
  }, [selectedModel]);

  const getSelectedModelInfo = () => {
    return modelsToUse.find(m => m.id === selectedModel) || modelsToUse[0];
  };

  // Show welcome screen if enabled
  if (showWelcome) {
    return (
      <div className="ai-pane-panel welcome-mode">
        <WelcomeScreen
          onNewFile={() => {
            // Trigger file creation
            const event = new CustomEvent('nava:new-file');
            window.dispatchEvent(event);
            setShowWelcome(false);
            localStorage.setItem('nava_ai_has_messages', 'true');
          }}
          onOpenFile={() => {
            const event = new CustomEvent('nava:open-file');
            window.dispatchEvent(event);
            setShowWelcome(false);
            localStorage.setItem('nava_ai_has_messages', 'true');
          }}
          onStartAgent={() => {
            setShowWelcome(false);
            localStorage.setItem('nava_ai_has_messages', 'true');
            // Focus on input
            setTimeout(() => {
              const input = document.querySelector('.ai-input') as HTMLTextAreaElement;
              input?.focus();
            }, 100);
          }}
          onCloneRepo={() => {
            const event = new CustomEvent('nava:clone-repo');
            window.dispatchEvent(event);
            setShowWelcome(false);
            localStorage.setItem('nava_ai_has_messages', 'true');
          }}
          onConnect={() => {
            const event = new CustomEvent('nava:open-settings');
            window.dispatchEvent(event);
            setShowWelcome(false);
            localStorage.setItem('nava_ai_has_messages', 'true');
          }}
          onClose={() => {
            setShowWelcome(false);
            localStorage.setItem('nava_ai_has_messages', 'true');
          }}
        />
      </div>
    );
  }

  return (
    <div className="ai-pane-panel">
      <div className="ai-pane-header">
        <div className="ai-pane-title">
          <span className="ai-icon">🤖</span>
          <span>NAVΛ AI Assistant</span>
          {authService.getCurrentUser() && (
            <span className="ai-user-badge" title={authService.getCurrentUser()?.email}>
              {authService.getCurrentUser()?.name || authService.getCurrentUser()?.email?.split('@')[0]}
            </span>
          )}
        </div>
        <div className="ai-controls">
          <select 
            value={selectedModel} 
            onChange={(e) => setSelectedModel(e.target.value)}
            className="model-selector"
            title={getSelectedModelInfo().description}
          >
            {modelsToUse.map((model) => (
              <option key={model.id} value={model.id}>
                {model.name}
              </option>
            ))}
          </select>
          <button 
            className="settings-btn"
            onClick={() => setShowSettings(!showSettings)}
            title="API Settings"
          >
            ⚙️
          </button>
        </div>
        <div className="ai-status">
          {(() => {
            // Check if backend is available using LLM Hub
            const selectedModelInfo = modelsToUse.find(m => m.id === selectedModel);
            const provider = selectedModelInfo?.provider || 'gemini';
            
            // Handle NAVA Local model specially
            let backendId: string;
            if (selectedModel === 'nava-local') {
              backendId = 'nava-local';
            } else {
              backendId = getBackendIdFromModel(selectedModel, provider);
            }
            
            const backend = backendRegistry.getBackend(backendId);
            const isBackendAvailable = backend?.isAvailable() || false;
            
            // Always show ready if free LLM is available (works without API keys)
            const isFreeLLMAvailable = true; // Free LLM always available as fallback
            const isReady = isBackendAvailable || isFreeLLMAvailable;
            
            // Get backend display name
            const backendName = backend?.displayName || (isBackendAvailable ? 'API' : 'Free');
            
            return (
              <>
                <span className={`status-indicator ${isThinking ? 'thinking' : isReady ? 'ready' : 'warning'}`}></span>
                <span className="status-text">
                  {isThinking ? 'Thinking...' : isBackendAvailable ? `Ready (${backendName})` : isFreeLLMAvailable ? 'Ready (Free)' : 'No API Key'}
                </span>
              </>
            );
          })()}
        </div>
      </div>

      {showSettings && (
        <div className="ai-settings-panel">
          <div className="settings-header">
            <h3>🔑 API Settings</h3>
            <button onClick={() => setShowSettings(false)} className="close-btn">✕</button>
          </div>
          <div className="settings-content">
            {/* Gemini API Settings */}
            <div style={{ marginBottom: '20px' }}>
              <div style={{ 
                background: 'rgba(66, 133, 244, 0.1)', 
                border: '1px solid rgba(66, 133, 244, 0.3)', 
                borderRadius: '6px', 
                padding: '12px', 
                marginBottom: '12px',
                fontSize: '13px',
                color: '#4285f4'
              }}>
                <strong>🚀 Gemini API (Default - FREE)</strong><br />
                Get your FREE API key from <a href="https://ai.google.dev/" target="_blank" rel="noopener noreferrer" style={{color: '#4285f4', textDecoration: 'underline'}}>Google AI Studio</a> using account: <strong>F.Van-Laarhoven2@newcastle.ac.uk</strong>
              </div>
              {isGeminiKeyFromEnv && (
                <div style={{
                  marginBottom: '8px',
                  padding: '8px 12px',
                  background: 'rgba(66, 133, 244, 0.15)',
                  border: '1px solid rgba(66, 133, 244, 0.3)',
                  borderRadius: '4px',
                  fontSize: '12px',
                  color: '#4285f4'
                }}>
                  ✓ Using API key from .env file (VITE_GEMINI_API_KEY)
                </div>
              )}
              <label htmlFor="gemini-api-key">
                Gemini API Key
                {!isGeminiKeyFromEnv && (
                  <a href="https://ai.google.dev/" target="_blank" rel="noopener noreferrer" className="get-key-link">
                    Get FREE key →
                  </a>
                )}
              </label>
              <input
                id="gemini-api-key"
                type="password"
                value={geminiApiKey}
                onChange={(e) => saveGeminiApiKey(e.target.value)}
                placeholder={isGeminiKeyFromEnv ? "Loaded from .env file" : "AIza..."}
                className="api-key-input"
                disabled={isGeminiKeyFromEnv}
                title={isGeminiKeyFromEnv ? "API key is loaded from .env file. Edit .env to change it." : ""}
              />
              {geminiApiKey && !isGeminiKeyFromEnv && (
                <div style={{
                  marginTop: '8px',
                  padding: '8px 12px',
                  background: 'rgba(0, 255, 0, 0.15)',
                  border: '1px solid rgba(0, 255, 0, 0.3)',
                  borderRadius: '4px',
                  fontSize: '12px',
                  color: '#00ff00'
                }}>
                  ✓ Gemini API Key Connected (from localStorage)!
                </div>
              )}
              {!isGeminiKeyFromEnv && (
                <div style={{
                  marginTop: '8px',
                  padding: '8px 12px',
                  background: 'rgba(255, 255, 255, 0.05)',
                  border: '1px solid rgba(255, 255, 255, 0.1)',
                  borderRadius: '4px',
                  fontSize: '11px',
                  color: '#88a2bf'
                }}>
                  💡 Tip: Add VITE_GEMINI_API_KEY to .env file for automatic loading
                </div>
              )}
            </div>

            {/* OpenRouter API Settings */}
            <div style={{ marginBottom: '20px' }}>
            <div style={{ 
              background: 'rgba(0, 255, 0, 0.1)', 
              border: '1px solid rgba(0, 255, 0, 0.3)', 
              borderRadius: '6px', 
              padding: '12px', 
                marginBottom: '12px',
              fontSize: '13px',
              color: '#00ff00'
            }}>
                <strong>💡 OpenRouter API</strong><br />
                Access to 100+ AI models including Claude 3.5 Sonnet, GPT-4o, Llama, and more!
            </div>
              {isOpenRouterKeyFromEnv && (
                <div style={{
                  marginBottom: '8px',
                  padding: '8px 12px',
                  background: 'rgba(0, 255, 0, 0.15)',
                  border: '1px solid rgba(0, 255, 0, 0.3)',
                  borderRadius: '4px',
                  fontSize: '12px',
                  color: '#00ff00'
                }}>
                  ✓ Using API key from .env file (VITE_OPENROUTER_API_KEY)
                </div>
              )}
              <label htmlFor="openrouter-api-key">
                OpenRouter API Key
                {!isOpenRouterKeyFromEnv && (
              <a href="https://openrouter.ai/keys" target="_blank" rel="noopener noreferrer" className="get-key-link">
                Get FREE key →
              </a>
                )}
            </label>
            <input
                id="openrouter-api-key"
              type="password"
                value={openRouterApiKey}
                onChange={(e) => saveOpenRouterApiKey(e.target.value)}
                placeholder={isOpenRouterKeyFromEnv ? "Loaded from .env file" : "sk-or-v1-..."}
              className="api-key-input"
                disabled={isOpenRouterKeyFromEnv}
                title={isOpenRouterKeyFromEnv ? "API key is loaded from .env file. Edit .env to change it." : ""}
            />
              {openRouterApiKey && !isOpenRouterKeyFromEnv && (
              <div style={{
                marginTop: '8px',
                padding: '8px 12px',
                background: 'rgba(0, 255, 0, 0.15)',
                border: '1px solid rgba(0, 255, 0, 0.3)',
                borderRadius: '4px',
                fontSize: '12px',
                color: '#00ff00'
              }}>
                  ✓ OpenRouter API Key Connected (from localStorage)!
              </div>
            )}
              {!isOpenRouterKeyFromEnv && (
                <div style={{
                  marginTop: '8px',
                  padding: '8px 12px',
                  background: 'rgba(255, 255, 255, 0.05)',
                  border: '1px solid rgba(255, 255, 255, 0.1)',
                  borderRadius: '4px',
                  fontSize: '11px',
                  color: '#88a2bf'
                }}>
                  💡 Tip: Add VITE_OPENROUTER_API_KEY to .env file for automatic loading
                </div>
              )}
            </div>

            <div className="model-info">
              <strong>Selected Model: {getSelectedModelInfo().name}</strong>
              <p>{getSelectedModelInfo().description}</p>
              <small>Context Window: {getSelectedModelInfo().contextLength.toLocaleString()} tokens • Provider: {getSelectedModelInfo().provider === 'gemini' ? 'Gemini API' : 'OpenRouter'}</small>
            </div>
            {!geminiApiKey && !openRouterApiKey && (
              <div className="warning-message">
                ⚠️ Without an API key, you'll get simulated responses. Add your Gemini API key (FREE) or OpenRouter API key above to unlock real AI models.
              </div>
            )}
          </div>
        </div>
      )}

      <div className="ai-messages">
        {messages.map((message) => (
          <div key={message.id} className={`ai-message ${message.role}`}>
            <div className="message-avatar">
              {message.role === 'user' ? '👤' : '🤖'}
            </div>
            <div className="message-content">
              <div className="message-header">
                <span className="message-role">
                  {message.role === 'user' ? 'You' : 'AI Assistant'}
                </span>
                <span className="message-time">
                  {message.timestamp.toLocaleTimeString([], { 
                    hour: '2-digit', 
                    minute: '2-digit' 
                  })}
                </span>
              </div>
              <div className="message-text">
                {message.files && message.files.length > 0 && (
                  <div className="message-files">
                    {message.files.map(file => (
                      <div key={file.id} className="message-file-item">
                        {file.preview && file.type.startsWith('image/') ? (
                          <img src={file.preview} alt={file.name} className="message-file-image" />
                        ) : (
                          <div className="message-file-icon">
                            {getFileIcon(file)}
                            <span className="message-file-name">{file.name}</span>
                            <span className="message-file-size">{formatFileSize(file.size)}</span>
                          </div>
                        )}
                      </div>
                    ))}
                  </div>
                )}
                {message.content.split('\n').map((line, i) => (
                  <React.Fragment key={i}>
                    {line}
                    {i < message.content.split('\n').length - 1 && <br />}
                  </React.Fragment>
                ))}
              </div>
            </div>
          </div>
        ))}
        {isThinking && (
          <div className="ai-message assistant thinking-indicator">
            <div className="message-avatar">🤖</div>
            <div className="thinking-dots">
              <span></span>
              <span></span>
              <span></span>
            </div>
          </div>
        )}
        <div ref={messagesEndRef} />
      </div>

      <div className="ai-input-area">
        {/* Symbol Toolbar - Compact with Voice Controls */}
        <div className="symbol-toolbar-compact">
          {/* NAVA Framework Symbols - Production Ready */}
          <button 
            onClick={() => insertNAVASymbol('⋋', '⋋')} 
            title="⋋ Van Laarhoven Lambda - Core NAVA navigation operator (e.g., navigate_to⋋, find_optimal_path⋋)" 
            className="symbol-btn-sm nava-symbol"
          >
            ⋋
          </button>
          <button 
            onClick={() => insertNAVASymbol('→', '→⋋')} 
            title="→ Navigation Arrow - Directional navigation operator (e.g., →⋋ for east navigation)" 
            className="symbol-btn-sm nava-symbol"
          >
            →
          </button>
          <button 
            onClick={() => insertNAVASymbol('∇', '∇⋋')} 
            title="∇ Nabla/Gradient - Navigation gradient operator (e.g., ∇⋋ for gradient flow)" 
            className="symbol-btn-sm nava-symbol"
          >
            ∇
          </button>
          <button 
            onClick={() => insertNAVASymbol('∫', '∫⋋')} 
            title="∫ Integral - Navigation path integral (e.g., ∫⋋ for path integration)" 
            className="symbol-btn-sm nava-symbol"
          >
            ∫
          </button>
          <button 
            onClick={() => insertNAVASymbol('φ', 'φ')} 
            title="φ Phi - Phase angle in navigation calculus (used in energy landscapes)" 
            className="symbol-btn-sm nava-symbol"
          >
            φ
          </button>
          
          {/* Voice Controls and Utilities */}
          <div style={{ marginLeft: 'auto', display: 'flex', gap: '6px', alignItems: 'center' }}>
            {/* Web Browser Button */}
            <button
              onClick={() => {
                // Open web browser/search capability
                const searchQuery = input.trim();
                if (searchQuery) {
                  // Check if it's a URL
                  if (searchQuery.startsWith('http://') || searchQuery.startsWith('https://') || searchQuery.startsWith('www.')) {
                    const url = searchQuery.startsWith('www.') ? `https://${searchQuery}` : searchQuery;
                    window.open(url, '_blank');
                  } else {
                    // Search on web using Google
                    const searchUrl = `https://www.google.com/search?q=${encodeURIComponent(searchQuery)}`;
                    window.open(searchUrl, '_blank');
                  }
                } else {
                  // If no input, prompt for search or URL
                  const query = prompt('Enter search query or URL:');
                  if (query) {
                    if (query.startsWith('http://') || query.startsWith('https://') || query.startsWith('www.')) {
                      const url = query.startsWith('www.') ? `https://${query}` : query;
                      window.open(url, '_blank');
                    } else {
                      const searchUrl = `https://www.google.com/search?q=${encodeURIComponent(query)}`;
                      window.open(searchUrl, '_blank');
                    }
                  }
                }
              }}
              title="Web Browser - Search web or open URL (uses current input or prompts)"
              className="symbol-btn-sm voice-control-btn"
            >
              <Globe size={16} />
            </button>
            
            <button
              onClick={() => {
                const enabled = voiceService.toggle();
                setIsVoiceEnabled(enabled);
              }}
              title={isVoiceEnabled ? 'Disable Voice' : 'Enable Voice'}
              className="symbol-btn-sm voice-control-btn"
            >
              {isVoiceEnabled ? <Volume2 size={16} /> : <VolumeX size={16} />}
            </button>
            
            <button
              onClick={() => {
                if (isListening) {
                  voiceService.stopListening();
                  setIsListening(false);
                } else {
                  voiceService.startListening((command) => {
                    setInput(command);
                  });
                  setIsListening(true);
                }
              }}
              disabled={!isVoiceEnabled}
              title={isListening ? 'Stop Listening' : 'Start Voice Commands'}
              className={`symbol-btn-sm voice-control-btn ${isListening ? 'listening' : ''}`}
            >
              {isListening ? <Mic size={16} /> : <MicOff size={16} />}
            </button>
            
            <button
              onClick={() => setShowSettings(!showSettings)}
              title="Settings - Configure API keys and models"
              className={`symbol-btn-sm voice-control-btn ${showSettings ? 'active' : ''}`}
            >
              <SettingsIcon size={16} />
            </button>
          </div>
        </div>

        {/* Uploaded Files Display */}
        {uploadedFiles.length > 0 && (
          <div className="uploaded-files-container">
            {uploadedFiles.map(file => (
              <div key={file.id} className="uploaded-file-chip">
                <div className="file-icon">{getFileIcon(file)}</div>
                {file.preview ? (
                  <img src={file.preview} alt={file.name} className="file-preview-thumb" />
                ) : null}
                <div className="file-info">
                  <div className="file-name" title={file.name}>{file.name}</div>
                  <div className="file-size">{formatFileSize(file.size)}</div>
                </div>
                <button 
                  onClick={() => removeFile(file.id)}
                  className="remove-file-btn"
                  title="Remove file"
                >
                  <X size={14} />
                </button>
              </div>
            ))}
          </div>
        )}

        {/* Hidden File Input */}
        <input
          ref={fileInputRef}
          type="file"
          multiple
          accept=".pdf,.csv,.txt,.doc,.docx,.xls,.xlsx,.ppt,.pptx,.odt,.ods,.odp,.jpg,.jpeg,.png,.gif,.webp,.svg,.bmp,.ico,.tiff,.tif,.glb,.gltf,.obj,.fbx,.dae,.3ds,.blend,.mp3,.wav,.ogg,.flac,.aac,.m4a,.wma,.opus,.mp4,.webm,.mov,.avi,.mkv,.flv,.wmv,.m4v,.py,.js,.ts,.jsx,.tsx,.json,.xml,.html,.css,.scss,.sass,.less,.yaml,.yml,.toml,.ini,.conf,.config,.log,.sh,.bash,.zsh,.fish,.ps1,.bat,.cmd,.ipynb,.r,.R,.sql,.pl,.pm,.rb,.go,.rs,.java,.kt,.scala,.clj,.hs,.elm,.ex,.exs,.ml,.mli,.fs,.fsx,.vb,.cs,.cpp,.cxx,.cc,.c,.h,.hpp,.hxx,.swift,.m,.mm,.dart,.lua,.vim,.vimrc,.zip,.rar,.7z,.tar,.gz,.bz2,.xz,application/pdf,application/msword,application/vnd.openxmlformats-officedocument.wordprocessingml.document,application/vnd.ms-excel,application/vnd.openxmlformats-officedocument.spreadsheetml.sheet,application/vnd.ms-powerpoint,application/vnd.openxmlformats-officedocument.presentationml.presentation,image/*,video/*,audio/*,text/*,application/json,application/xml"
          onChange={handleFileUpload}
          style={{ display: 'none' }}
        />

        {/* Perplexity-Style Input */}
        <div className="prompt-input-container">
          <button
            onClick={() => fileInputRef.current?.click()}
            className="paperclip-button"
            title="Upload files (images, documents, code, videos, 3D models, etc.)"
          >
            <Paperclip size={20} />
          </button>
          <textarea
            value={input}
            onChange={(e) => setInput(e.target.value)}
            onKeyDown={handleKeyDown}
            placeholder="Ask me anything about NAVΛ or VNC..."
            className="prompt-textarea"
            rows={1}
            onInput={(e) => {
              const target = e.target as HTMLTextAreaElement;
              target.style.height = 'auto';
              target.style.height = Math.min(target.scrollHeight, 150) + 'px';
            }}
          />
          <button 
            onClick={handleSend} 
            disabled={!input.trim() || isThinking}
            className="send-button-modern"
            title={isThinking ? "Thinking..." : "Send message (Enter)"}
          >
            {isThinking ? (
              <span className="spinner-pulse">⏳</span>
            ) : (
              <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                <path d="M22 2L11 13M22 2l-7 20-4-9-9-4 20-7z" />
              </svg>
            )}
          </button>
        </div>
        
        {/* Helper Text */}
        {!geminiApiKey && !openRouterApiKey && (
          <div className="input-helper">
            💡 Local code generator active • <span onClick={() => setShowSettings(true)} className="link-text">Add Gemini API key (FREE)</span> or OpenRouter key for full AI
          </div>
        )}
      </div>
    </div>
  );
};

