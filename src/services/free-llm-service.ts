/**
 * Free LLM Service
 * Provides free AI responses without requiring API keys
 * Uses Hugging Face Inference API (free tier) and other free services
 */

export interface FreeLLMResponse {
  text: string;
  model: string;
  error?: string;
}

class FreeLLMService {
  private readonly HUGGINGFACE_API_URL = 'https://api-inference.huggingface.co/models';
  private readonly GROQ_API_URL = 'https://api.groq.com/openai/v1/chat/completions';
  
  // Free models that don't require API keys (public endpoints)
  private readonly FREE_MODELS = [
    {
      id: 'huggingface-mistral-7b',
      name: 'Mistral 7B (Free)',
      endpoint: `${this.HUGGINGFACE_API_URL}/mistralai/Mistral-7B-Instruct-v0.2`,
      type: 'huggingface',
    },
    {
      id: 'huggingface-llama-7b',
      name: 'Llama 7B (Free)',
      endpoint: `${this.HUGGINGFACE_API_URL}/meta-llama/Llama-2-7b-chat-hf`,
      type: 'huggingface',
    },
    {
      id: 'huggingface-codellama',
      name: 'CodeLlama (Free)',
      endpoint: `${this.HUGGINGFACE_API_URL}/codellama/CodeLlama-7b-Instruct-hf`,
      type: 'huggingface',
    },
  ];

  /**
   * Get a free AI response without requiring API keys
   */
  async getFreeResponse(prompt: string, conversationHistory: Array<{ role: string; content: string }> = []): Promise<FreeLLMResponse> {
    // Try Hugging Face first (completely free, no API key needed for public models)
    try {
      return await this.tryHuggingFace(prompt, conversationHistory);
    } catch (error) {
      console.warn('[Free LLM] Hugging Face failed, trying fallback:', error);
    }

    // Fallback to a simple but helpful response generator
    return this.generateFallbackResponse(prompt, conversationHistory);
  }

  /**
   * Try Hugging Face Inference API (free, no API key required for public models)
   */
  private async tryHuggingFace(prompt: string, conversationHistory: Array<{ role: string; content: string }>): Promise<FreeLLMResponse> {
    const model = this.FREE_MODELS[0]; // Use Mistral 7B
    
    // Format conversation for the model
    const conversationText = conversationHistory
      .slice(-5) // Last 5 messages
      .map(msg => `${msg.role === 'user' ? 'User' : 'Assistant'}: ${msg.content}`)
      .join('\n\n');
    
    const fullPrompt = conversationText 
      ? `${conversationText}\n\nUser: ${prompt}\n\nAssistant:`
      : `You are a helpful AI coding assistant. Answer the following question:\n\n${prompt}\n\nAnswer:`;

    try {
      const response = await fetch(model.endpoint, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          inputs: fullPrompt,
          parameters: {
            max_new_tokens: 512,
            temperature: 0.7,
            top_p: 0.9,
            return_full_text: false,
          },
        }),
      });

      if (!response.ok) {
        // If model is loading, wait and retry
        if (response.status === 503) {
          const retryAfter = response.headers.get('Retry-After');
          if (retryAfter) {
            await new Promise(resolve => setTimeout(resolve, parseInt(retryAfter) * 1000));
            return this.tryHuggingFace(prompt, conversationHistory);
          }
        }
        throw new Error(`Hugging Face API error: ${response.statusText}`);
      }

      const data = await response.json();
      
      // Handle different response formats
      let text = '';
      if (Array.isArray(data)) {
        text = data[0]?.generated_text || data[0]?.text || '';
      } else if (data.generated_text) {
        text = data.generated_text;
      } else if (data[0]?.generated_text) {
        text = data[0].generated_text;
      } else if (typeof data === 'string') {
        text = data;
      }

      // Clean up the response (remove the prompt if it's included)
      if (text.includes(fullPrompt)) {
        text = text.replace(fullPrompt, '').trim();
      }
      if (text.includes('Assistant:')) {
        text = text.split('Assistant:').pop()?.trim() || text;
      }

      return {
        text: text || 'I received an empty response. Please try rephrasing your question.',
        model: model.name,
      };
    } catch (error: any) {
      throw new Error(`Hugging Face error: ${error.message}`);
    }
  }

  /**
   * Generate a helpful fallback response when all free APIs fail
   */
  private generateFallbackResponse(prompt: string, conversationHistory: Array<{ role: string; content: string }>): FreeLLMResponse {
    const lowerPrompt = prompt.toLowerCase();
    
    // Provide helpful responses based on common queries
    let response = '';

    if (lowerPrompt.includes('hello') || lowerPrompt.includes('hi') || lowerPrompt.includes('hey')) {
      response = 'Hello! I\'m your NAVΛ AI Assistant. I\'m here to help you with coding, debugging, and answering questions. How can I assist you today?';
    } else if (lowerPrompt.includes('weather')) {
      response = 'I don\'t have access to real-time weather data, but I can help you:\n\n1. Build a weather app using APIs like OpenWeatherMap\n2. Create a weather widget for your project\n3. Help with weather-related code\n\nWould you like help with any of these?';
    } else if (lowerPrompt.includes('code') || lowerPrompt.includes('generate') || lowerPrompt.includes('create') || lowerPrompt.includes('write')) {
      response = `I can help you with code! Here's what I can assist with:\n\n- Writing code in multiple languages (TypeScript, Python, Rust, SQL, R, NAVΛ)\n- Debugging and fixing errors\n- Explaining code concepts\n- Generating code from descriptions\n\nFor the best experience, you can add an API key in settings, but I'll do my best to help you now. What specific code do you need?`;
    } else if (lowerPrompt.includes('error') || lowerPrompt.includes('bug') || lowerPrompt.includes('fix')) {
      response = 'I can help you debug! Please share:\n\n1. The error message\n2. The code that\'s causing the issue\n3. What you were trying to accomplish\n\nI\'ll help you identify and fix the problem.';
    } else if (lowerPrompt.includes('explain') || lowerPrompt.includes('what is') || lowerPrompt.includes('how does')) {
      response = 'I\'d be happy to explain! Could you provide more details about what you\'d like me to explain? For example:\n\n- A specific programming concept\n- How a piece of code works\n- A technology or framework\n- A NAVΛ/VNC concept\n\nI\'ll provide a clear, detailed explanation.';
    } else if (lowerPrompt.includes('help') || lowerPrompt.includes('how to')) {
      response = 'I\'m here to help! I can assist with:\n\n✅ **Coding**: Write, debug, and explain code\n✅ **NAVΛ/VNC**: Help with navigation calculus\n✅ **Projects**: Scaffold and structure projects\n✅ **Learning**: Explain concepts and best practices\n✅ **Troubleshooting**: Fix errors and optimize code\n\nWhat would you like help with? Be specific and I\'ll provide detailed assistance.';
    } else {
      // Generic helpful response
      response = `I understand you're asking about: "${prompt}"\n\nI'm your NAVΛ AI Assistant, and I'm here to help! While I work best with an API key configured, I can still assist you with:\n\n- **Coding questions**: Ask me about programming, debugging, or code generation\n- **NAVΛ/VNC**: Questions about navigation calculus and the NAVΛ framework\n- **Technical help**: Explaining concepts, best practices, and troubleshooting\n- **Project assistance**: Help structuring and organizing your codebase\n\nFor the most accurate and detailed responses, consider adding a free Gemini API key (from https://ai.google.dev/) in the settings. But I'll do my best to help you right now!\n\nCould you rephrase your question or be more specific about what you need?`;
    }

    return {
      text: response,
      model: 'Fallback Assistant',
    };
  }

  /**
   * Check if free LLM service is available
   */
  async isAvailable(): Promise<boolean> {
    try {
      // Quick check if Hugging Face is accessible
      const response = await fetch(`${this.HUGGINGFACE_API_URL}/mistralai/Mistral-7B-Instruct-v0.2`, {
        method: 'HEAD',
      });
      return response.ok || response.status === 503; // 503 means model is loading but available
    } catch {
      return false;
    }
  }
}

export const freeLLMService = new FreeLLMService();

