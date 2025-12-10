/**
 * AI Code Generator Service
 * Generates code based on conversational prompts (like Cursor)
 */

import { modelConfigService } from './model-config-service';
import { authService } from './auth-service';

export interface CodeGenerationRequest {
  prompt: string;
  context?: {
    currentFile?: string;
    currentCode?: string;
    language?: string;
    framework?: string;
    cursorPosition?: { line: number; column: number };
  };
}

export interface CodeGenerationResult {
  code: string;
  explanation?: string;
  language?: string;
  framework?: string;
  files?: Array<{
    path: string;
    content: string;
  }>;
}

class AICodeGeneratorService {
  /**
   * Generate code based on a conversational prompt
   */
  async generateCode(request: CodeGenerationRequest): Promise<CodeGenerationResult> {
    const user = authService.getCurrentUser();
    const defaultModel = modelConfigService.getDefaultModel();
    const model = modelConfigService.getModels().find(m => m.id === defaultModel);
    
    if (!model) {
      throw new Error('No AI model configured');
    }

    // Build the prompt with context
    const systemPrompt = this.buildSystemPrompt(request.context);
    const userPrompt = this.buildUserPrompt(request);

    // Call the AI model
    const response = await this.callAIModel(model.id, systemPrompt, userPrompt);
    
    // Parse the response
    return this.parseAIResponse(response, request.context);
  }

  /**
   * Build system prompt with context
   */
  private buildSystemPrompt(context?: CodeGenerationRequest['context']): string {
    let prompt = `You are an expert code assistant for NAVΛ Studio IDE. Generate production-ready code based on user requests.

Context:
- Current file: ${context?.currentFile || 'new file'}
- Language: ${context?.language || 'auto-detect'}
- Framework: ${context?.framework || 'none specified'}

Instructions:
1. Generate clean, production-ready code
2. Include necessary imports
3. Add helpful comments
4. Follow best practices for the specified language/framework
5. If multiple files are needed, format as JSON with "files" array
6. Always return valid code that can be directly used

Format your response as:
- If single file: Return the code directly
- If multiple files: Return JSON with {"files": [{"path": "...", "content": "..."}]}
`;

    if (context?.currentCode) {
      prompt += `\nCurrent code context:\n\`\`\`\n${context.currentCode.substring(0, 2000)}\n\`\`\`\n`;
    }

    return prompt;
  }

  /**
   * Build user prompt
   */
  private buildUserPrompt(request: CodeGenerationRequest): string {
    let prompt = request.prompt;

    if (request.context?.cursorPosition) {
      prompt += `\n\nCursor position: Line ${request.context.cursorPosition.line}, Column ${request.context.cursorPosition.column}`;
    }

    return prompt;
  }

  /**
   * Call the AI model
   */
  private async callAIModel(
    modelId: string,
    systemPrompt: string,
    userPrompt: string
  ): Promise<string> {
    const model = modelConfigService.getModels().find(m => m.id === modelId);
    if (!model) {
      throw new Error(`Model ${modelId} not found`);
    }

    const provider = model.provider;
    const apiKey = modelConfigService.getProviderApiKey(provider);

    if (!apiKey && provider !== 'mcp-github' && provider !== 'mcp-slack' && provider !== 'mcp-linear') {
      throw new Error(`API key not configured for ${provider}`);
    }

    // Gemini API
    if (provider === 'gemini') {
      return this.callGeminiAPI(apiKey, systemPrompt, userPrompt, model.apiEndpoint || model.id);
    }

    // OpenRouter API
    if (provider === 'openrouter') {
      return this.callOpenRouterAPI(apiKey, systemPrompt, userPrompt, model.id);
    }

    throw new Error(`Unsupported provider: ${provider}`);
  }

  /**
   * Call Gemini API
   */
  private async callGeminiAPI(
    apiKey: string,
    systemPrompt: string,
    userPrompt: string,
    modelId: string
  ): Promise<string> {
    const endpoint = `https://generativelanguage.googleapis.com/v1beta/models/${modelId}:generateContent?key=${apiKey}`;

    const response = await fetch(endpoint, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        contents: [
          {
            parts: [
              { text: systemPrompt },
              { text: userPrompt },
            ],
          },
        ],
        generationConfig: {
          temperature: 0.7,
          topK: 40,
          topP: 0.95,
          maxOutputTokens: 8192,
        },
      }),
    });

    if (!response.ok) {
      throw new Error(`Gemini API error: ${response.statusText}`);
    }

    const data = await response.json();
    return data.candidates[0]?.content?.parts[0]?.text || '';
  }

  /**
   * Call OpenRouter API
   */
  private async callOpenRouterAPI(
    apiKey: string,
    systemPrompt: string,
    userPrompt: string,
    modelId: string
  ): Promise<string> {
    const response = await fetch('https://openrouter.ai/api/v1/chat/completions', {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
        'Authorization': `Bearer ${apiKey}`,
        'HTTP-Referer': window.location.origin,
        'X-Title': 'NAVΛ Studio IDE',
      },
      body: JSON.stringify({
        model: modelId,
        messages: [
          { role: 'system', content: systemPrompt },
          { role: 'user', content: userPrompt },
        ],
        temperature: 0.7,
        max_tokens: 8192,
      }),
    });

    if (!response.ok) {
      throw new Error(`OpenRouter API error: ${response.statusText}`);
    }

    const data = await response.json();
    return data.choices[0]?.message?.content || '';
  }

  /**
   * Parse AI response
   */
  private parseAIResponse(
    response: string,
    context?: CodeGenerationRequest['context']
  ): CodeGenerationResult {
    // Try to parse as JSON (multiple files)
    try {
      const json = JSON.parse(response);
      if (json.files && Array.isArray(json.files)) {
        return {
          code: json.files[0]?.content || '',
          explanation: json.explanation,
          language: context?.language,
          framework: context?.framework,
          files: json.files,
        };
      }
    } catch {
      // Not JSON, treat as single code block
    }

    // Extract code from markdown code blocks
    const codeBlockRegex = /```(?:\w+)?\n([\s\S]*?)```/;
    const match = response.match(codeBlockRegex);
    
    if (match) {
      return {
        code: match[1],
        explanation: response.replace(codeBlockRegex, '').trim() || undefined,
        language: context?.language,
        framework: context?.framework,
      };
    }

    // Return as-is if no code blocks found
    return {
      code: response.trim(),
      language: context?.language,
      framework: context?.framework,
    };
  }
}

export const aiCodeGeneratorService = new AICodeGeneratorService();

