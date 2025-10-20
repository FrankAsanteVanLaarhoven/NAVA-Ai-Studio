import React, { useState, useRef, useEffect } from 'react';
import './AIPanePanel.css';
import { mcpCodeGenerator } from '../../services/mcp-code-generator';
import { projectScaffolder } from '../../services/project-scaffolder';
import { Mic, MicOff, Volume2, VolumeX, Settings as SettingsIcon, Paperclip, X, File, Image as ImageIcon, FileText, Video, FileCode } from 'lucide-react';
import { voiceService } from '../../services/voice-service';

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

export const AIPanePanel: React.FC = () => {
  // Available OpenRouter models (Latest as of Oct 2025)
  const availableModels: AIModel[] = [
    // Anthropic Claude Models (Best for Code!)
    {
      id: 'anthropic/claude-3.5-sonnet',
      name: '✨ Claude 3.5 Sonnet',
      description: '⭐ BEST for coding! Most capable model for code generation',
      contextLength: 200000,
    },
    {
      id: 'anthropic/claude-3-opus',
      name: 'Claude 3 Opus',
      description: 'Most powerful - Best for complex reasoning',
      contextLength: 200000,
    },
    {
      id: 'anthropic/claude-3-sonnet',
      name: 'Claude 3 Sonnet',
      description: 'Balanced performance and speed',
      contextLength: 200000,
    },
    {
      id: 'anthropic/claude-3-haiku',
      name: 'Claude 3 Haiku',
      description: 'Fastest Claude model - Great for simple tasks',
      contextLength: 200000,
    },
    
    // OpenAI Models (Latest)
    {
      id: 'openai/gpt-4-turbo-preview',
      name: '🚀 GPT-4 Turbo',
      description: 'Latest GPT-4 with 128K context - Vision capable',
      contextLength: 128000,
    },
    {
      id: 'openai/gpt-4',
      name: 'GPT-4',
      description: 'OpenAI flagship model - Most capable GPT',
      contextLength: 8192,
    },
    {
      id: 'openai/gpt-4-32k',
      name: 'GPT-4 32K',
      description: 'GPT-4 with extended context',
      contextLength: 32768,
    },
    {
      id: 'openai/gpt-3.5-turbo',
      name: 'GPT-3.5 Turbo',
      description: 'Fast and cost-effective',
      contextLength: 16385,
    },
    {
      id: 'openai/gpt-3.5-turbo-16k',
      name: 'GPT-3.5 Turbo 16K',
      description: 'Extended context GPT-3.5',
      contextLength: 16385,
    },
    {
      id: 'openai/o1-preview',
      name: '🧠 OpenAI O1 Preview',
      description: 'Advanced reasoning model - Best for complex problems',
      contextLength: 128000,
    },
    {
      id: 'openai/o1-mini',
      name: 'OpenAI O1 Mini',
      description: 'Faster reasoning model',
      contextLength: 128000,
    },
    
    // Google Models
    {
      id: 'google/gemini-pro',
      name: 'Gemini Pro',
      description: 'Google\'s most capable model',
      contextLength: 32000,
    },
    {
      id: 'google/gemini-pro-vision',
      name: 'Gemini Pro Vision',
      description: 'Gemini with image understanding',
      contextLength: 32000,
    },
    {
      id: 'google/gemini-1.5-pro',
      name: '⚡ Gemini 1.5 Pro',
      description: 'Latest Gemini - 1M context window!',
      contextLength: 1000000,
    },
    {
      id: 'google/gemini-1.5-flash',
      name: 'Gemini 1.5 Flash',
      description: 'Fast multimodal model',
      contextLength: 1000000,
    },
    
    // Meta Llama Models (Open Source)
    {
      id: 'meta-llama/llama-3.1-405b-instruct',
      name: '🦙 Llama 3.1 405B',
      description: 'Largest open model - Extremely powerful!',
      contextLength: 128000,
    },
    {
      id: 'meta-llama/llama-3.1-70b-instruct',
      name: 'Llama 3.1 70B',
      description: 'High performance open model',
      contextLength: 128000,
    },
    {
      id: 'meta-llama/llama-3.1-8b-instruct',
      name: 'Llama 3.1 8B',
      description: 'Fast and efficient',
      contextLength: 128000,
    },
    {
      id: 'meta-llama/llama-3-70b-instruct',
      name: 'Llama 3 70B',
      description: 'Previous gen - Still powerful',
      contextLength: 8192,
    },
    
    // Mistral Models (European Excellence)
    {
      id: 'mistralai/mistral-large',
      name: '🇪🇺 Mistral Large',
      description: 'Top European model - Excellent for code',
      contextLength: 32000,
    },
    {
      id: 'mistralai/mistral-medium',
      name: 'Mistral Medium',
      description: 'Balanced performance',
      contextLength: 32000,
    },
    {
      id: 'mistralai/mixtral-8x7b-instruct',
      name: 'Mixtral 8x7B',
      description: 'MoE model - Fast and powerful',
      contextLength: 32000,
    },
    {
      id: 'mistralai/mixtral-8x22b',
      name: 'Mixtral 8x22B',
      description: 'Largest Mixtral - 141B params total',
      contextLength: 64000,
    },
    
    // Specialized Models
    {
      id: 'deepseek/deepseek-coder-33b-instruct',
      name: '💻 DeepSeek Coder 33B',
      description: 'Specialized for code generation',
      contextLength: 16000,
    },
    {
      id: 'cohere/command-r-plus',
      name: 'Command R+',
      description: 'Enterprise-grade model with RAG',
      contextLength: 128000,
    },
    {
      id: 'perplexity/llama-3.1-sonar-large-128k-online',
      name: '🔍 Perplexity Sonar',
      description: 'Online search-enhanced model',
      contextLength: 128000,
    },
    {
      id: 'anthropic/claude-2.1',
      name: 'Claude 2.1',
      description: 'Previous generation Claude',
      contextLength: 200000,
    },
  ];

  const [messages, setMessages] = useState<Message[]>([
    {
      id: '1',
      role: 'assistant',
      content: '👋 Hello! I\'m your NAVΛ AI Assistant. I can help you with:\n\n• Understanding Van Laarhoven Navigation Calculus\n• Writing and optimizing ⋋ expressions\n• Debugging navigation algorithms\n• Explaining mathematical concepts\n• Code suggestions and completions\n\nHow can I assist you today?',
      timestamp: new Date(),
    },
  ]);

  const [input, setInput] = useState('');
  const [isThinking, setIsThinking] = useState(false);
  const [selectedModel, setSelectedModel] = useState<string>(availableModels[0].id);
  const [apiKey, setApiKey] = useState<string>(localStorage.getItem('openrouter_api_key') || '');
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
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

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
      // Use OpenRouter API if key is provided
      if (apiKey) {
        const response = await fetch('https://openrouter.ai/api/v1/chat/completions', {
          method: 'POST',
          headers: {
            'Authorization': `Bearer ${apiKey}`,
            'HTTP-Referer': window.location.origin,
            'X-Title': 'NAVΛ Studio IDE',
            'Content-Type': 'application/json',
          },
          body: JSON.stringify({
            model: selectedModel,
            messages: [
              {
                role: 'system',
                content: 'You are an expert AI assistant specialized in Van Laarhoven Navigation Calculus (VNC). Help users understand and write VNC code, explain navigation concepts, and provide coding assistance. Use the ⋋ (Van Laarhoven Lambda) symbol when relevant.',
              },
              ...messages.slice(-5).map(m => ({
                role: m.role,
                content: m.content,
              })),
              {
                role: 'user',
                content: query,
              },
            ],
          }),
        });

        if (!response.ok) {
          throw new Error(`OpenRouter API error: ${response.statusText}`);
        }

        const data = await response.json();
        const aiContent = data.choices[0]?.message?.content || 'No response received.';

        const aiResponse: Message = {
          id: (Date.now() + 1).toString(),
          role: 'assistant',
          content: aiContent,
          timestamp: new Date(),
        };

        setMessages(prev => [...prev, aiResponse]);
      } else {
        // Use local code generator if no API key
        await new Promise(resolve => setTimeout(resolve, 800));

        // Check if user wants to generate a project
        const lowerQuery = query.toLowerCase();
        let aiContent = '';
        
        if (lowerQuery.includes('generate') || lowerQuery.includes('create') || lowerQuery.includes('build')) {
          // Use MCP Code Generator
          try {
            const project = await mcpCodeGenerator.generateProject({
              prompt: query,
              projectName: 'generated-project',
              includeDeployment: lowerQuery.includes('docker') || lowerQuery.includes('deploy'),
              includeTests: lowerQuery.includes('test'),
            });
            
            aiContent = `✅ **Project Generated!**\n\n**Name:** ${project.name}\n**Description:** ${project.description}\n\n**Files Created (${project.files.length}):**\n\n${project.files.map(f => `📄 \`${f.path}\` - ${f.description}`).join('\n')}\n\n**Dependencies:**\n${project.dependencies.map(d => `- ${d}`).join('\n')}\n\n---\n\n**First File Preview:**\n\n\`\`\`vnc\n${project.files[0]?.content.substring(0, 500)}...\n\`\`\`\n\n💡 **Next Steps:**\n1. Copy the code above\n2. Create files in your editor\n3. Click "⚙ Compile" to build\n4. Click "▶ Run" to execute\n\n**Would you like me to show you another file or explain the navigation mathematics?**`;
          } catch (error) {
            aiContent = `⚠️ Error generating project: ${error}\n\nLet me help you manually. What specific component would you like to start with?`;
          }
        } else {
          // Use template response for other queries
          aiContent = generateAIResponse(query);
        }

        const aiResponse: Message = {
          id: (Date.now() + 1).toString(),
          role: 'assistant',
          content: aiContent,
          timestamp: new Date(),
        };

        setMessages(prev => [...prev, aiResponse]);
      }
    } catch (error) {
      console.error('Error calling OpenRouter:', error);
      const errorMessage: Message = {
        id: (Date.now() + 1).toString(),
        role: 'assistant',
        content: `⚠️ Error: ${error instanceof Error ? error.message : 'Failed to get AI response'}. Please check your API key and try again.`,
        timestamp: new Date(),
      };
      setMessages(prev => [...prev, errorMessage]);
    }

    setIsThinking(false);
  };

  const generateAIResponse = (query: string): string => {
    const lowerQuery = query.toLowerCase();

    if (lowerQuery.includes('⋋') || lowerQuery.includes('lambda')) {
      return '**Van Laarhoven Lambda (⋋)**\n\nThe ⋋ operator represents a navigation transformation in VNC. Here\'s how it works:\n\n```vnc\n⋋ φ: M → N\n```\n\nThis defines a navigation mapping from manifold M to manifold N. You can use it to:\n\n1. Define navigation fields\n2. Compose transformations\n3. Optimize paths through geometric spaces\n\nWould you like me to show you a specific example?';
    }

    if (lowerQuery.includes('optimize') || lowerQuery.includes('performance')) {
      return '**Performance Optimization Tips**\n\n1. **Use Memoization**: Cache computed navigation paths\n2. **Vectorize Operations**: Leverage SIMD for manifold computations\n3. **Parallel Processing**: Split large navigation fields across threads\n4. **GPU Acceleration**: Use WASM-SIMD for browser-based calculations\n\nHere\'s an optimized pattern:\n\n```vnc\n@memoize\n⋋ fast_nav = parallel_map(nav_field)\n```';
    }

    if (lowerQuery.includes('error') || lowerQuery.includes('debug')) {
      return '**Debugging Navigation Code**\n\n🔍 Common issues:\n\n1. **Type Mismatches**: Ensure manifold dimensions align\n2. **Singularities**: Check for division by zero in metric tensors\n3. **Convergence**: Verify optimization parameters\n\nTry adding:\n```vnc\n@debug_trace\n⋋ my_navigation_fn\n```\n\nThis will show step-by-step execution.';
    }

    return `I understand you're asking about: "${query}"\n\nLet me help you with that. The Van Laarhoven Navigation Calculus provides powerful abstractions for:\n\n• **Geometric computations** on manifolds\n• **Path optimization** through configuration spaces\n• **Energy minimization** in navigation fields\n\nCould you provide more specific details about what you'd like to accomplish?`;
  };

  const handleKeyDown = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSend();
    }
  };

  const insertSymbol = (symbol: string) => {
    setInput(prev => prev + symbol);
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
      if (file.type.startsWith('image/')) {
        const reader = new FileReader();
        reader.onload = (e) => {
          fileData.preview = e.target?.result as string;
          fileData.content = e.target?.result;
        };
        reader.readAsDataURL(file);
      } else if (file.type.startsWith('text/') || 
                 file.type.includes('json') || 
                 file.type.includes('javascript') ||
                 file.type.includes('typescript') ||
                 file.name.endsWith('.csv') ||
                 file.name.endsWith('.md') ||
                 file.name.endsWith('.txt')) {
        const reader = new FileReader();
        reader.onload = (e) => {
          fileData.content = e.target?.result as string;
        };
        reader.readAsText(file);
      } else {
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

  const saveApiKey = (key: string) => {
    setApiKey(key);
    localStorage.setItem('openrouter_api_key', key);
  };

  const getSelectedModelInfo = () => {
    return availableModels.find(m => m.id === selectedModel) || availableModels[0];
  };

  return (
    <div className="ai-pane-panel">
      <div className="ai-pane-header">
        <div className="ai-pane-title">
          <span className="ai-icon">🤖</span>
          <span>NAVΛ AI Assistant</span>
        </div>
        <div className="ai-controls">
          <select 
            value={selectedModel} 
            onChange={(e) => setSelectedModel(e.target.value)}
            className="model-selector"
            title={getSelectedModelInfo().description}
          >
            {availableModels.map((model) => (
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
          <span className={`status-indicator ${isThinking ? 'thinking' : apiKey ? 'ready' : 'warning'}`}></span>
          <span className="status-text">{isThinking ? 'Thinking...' : apiKey ? 'Ready' : 'No API Key'}</span>
        </div>
      </div>

      {showSettings && (
        <div className="ai-settings-panel">
          <div className="settings-header">
            <h3>🔑 OpenRouter API Settings</h3>
            <button onClick={() => setShowSettings(false)} className="close-btn">✕</button>
          </div>
          <div className="settings-content">
            <div style={{ 
              background: 'rgba(0, 255, 0, 0.1)', 
              border: '1px solid rgba(0, 255, 0, 0.3)', 
              borderRadius: '6px', 
              padding: '12px', 
              marginBottom: '16px',
              fontSize: '13px',
              color: '#00ff00'
            }}>
              <strong>💡 Tip:</strong> OpenRouter gives you access to 100+ AI models including Claude 3.5 Sonnet, GPT-4, Gemini, Llama, and more!
            </div>
            <label htmlFor="api-key">
              API Key
              <a href="https://openrouter.ai/keys" target="_blank" rel="noopener noreferrer" className="get-key-link">
                Get FREE key →
              </a>
            </label>
            <input
              id="api-key"
              type="password"
              value={apiKey}
              onChange={(e) => saveApiKey(e.target.value)}
              placeholder="sk-or-v1-..."
              className="api-key-input"
            />
            {apiKey && (
              <div style={{
                marginTop: '8px',
                padding: '8px 12px',
                background: 'rgba(0, 255, 0, 0.15)',
                border: '1px solid rgba(0, 255, 0, 0.3)',
                borderRadius: '4px',
                fontSize: '12px',
                color: '#00ff00'
              }}>
                ✓ API Key Connected - All models unlocked!
              </div>
            )}
            <div className="model-info">
              <strong>Selected Model: {getSelectedModelInfo().name}</strong>
              <p>{getSelectedModelInfo().description}</p>
              <small>Context Window: {getSelectedModelInfo().contextLength.toLocaleString()} tokens</small>
            </div>
            {!apiKey && (
              <div className="warning-message">
                ⚠️ Without an API key, you'll get simulated responses. Add your OpenRouter API key above to unlock real AI models.
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
          <button onClick={() => insertSymbol('⋋')} title="⋋ Van Laarhoven Lambda" className="symbol-btn-sm">⋋</button>
          <button onClick={() => insertSymbol('→')} title="→ Arrow" className="symbol-btn-sm">→</button>
          <button onClick={() => insertSymbol('∇')} title="∇ Nabla/Gradient" className="symbol-btn-sm">∇</button>
          <button onClick={() => insertSymbol('∫')} title="∫ Integral" className="symbol-btn-sm">∫</button>
          <button onClick={() => insertSymbol('φ')} title="φ Phi" className="symbol-btn-sm">φ</button>
          
          {/* Voice Controls */}
          <div style={{ marginLeft: 'auto', display: 'flex', gap: '6px', alignItems: 'center' }}>
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
              onClick={() => setShowVoiceSettings(!showVoiceSettings)}
              title="Voice Settings"
              className="symbol-btn-sm voice-control-btn"
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
          accept="*/*"
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
        {!apiKey && (
          <div className="input-helper">
            💡 Local code generator active • <span onClick={() => setShowSettings(true)} className="link-text">Add API key</span> for full AI
          </div>
        )}
      </div>
    </div>
  );
};

