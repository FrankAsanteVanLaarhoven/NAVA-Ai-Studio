/**
 * NAVA Tools Service
 * Tool layer for IDE-aware operations: run_nava, compile, read_file, etc.
 */

export interface NAVAToolResult {
  success: boolean;
  output?: string;
  error?: string;
  exitCode?: number;
  metadata?: Record<string, any>;
}

export interface NAVATool {
  name: string;
  description: string;
  parameters: {
    name: string;
    type: string;
    description: string;
    required: boolean;
  }[];
}

class NAVAToolsService {
  /**
   * Available tools for NAVA assistant
   */
  getAvailableTools(): NAVATool[] {
    return [
      {
        name: 'run_nava',
        description: 'Execute NAVA code and return stdout, stderr, and exit code',
        parameters: [
          {
            name: 'code',
            type: 'string',
            description: 'NAVA/VΛNC code to execute',
            required: true,
          },
          {
            name: 'interpreter',
            type: 'string',
            description: 'Interpreter to use: tauri-backend, python, wasm, browser-sim',
            required: false,
          },
        ],
      },
      {
        name: 'compile_nava',
        description: 'Check if NAVA code compiles without errors',
        parameters: [
          {
            name: 'code',
            type: 'string',
            description: 'NAVA/VΛNC code to compile',
            required: true,
          },
        ],
      },
      {
        name: 'read_file',
        description: 'Read a file from the workspace',
        parameters: [
          {
            name: 'path',
            type: 'string',
            description: 'File path relative to workspace root',
            required: true,
          },
        ],
      },
      {
        name: 'write_file',
        description: 'Write content to a file in the workspace',
        parameters: [
          {
            name: 'path',
            type: 'string',
            description: 'File path relative to workspace root',
            required: true,
          },
          {
            name: 'content',
            type: 'string',
            description: 'File content to write',
            required: true,
          },
        ],
      },
      {
        name: 'list_files',
        description: 'List files in a directory',
        parameters: [
          {
            name: 'path',
            type: 'string',
            description: 'Directory path (default: workspace root)',
            required: false,
          },
        ],
      },
      {
        name: 'get_compiler_help',
        description: 'Get help for a specific compiler error',
        parameters: [
          {
            name: 'error_code',
            type: 'string',
            description: 'Compiler error code or message',
            required: true,
          },
        ],
      },
      {
        name: 'search_docs',
        description: 'Search NAVA documentation',
        parameters: [
          {
            name: 'query',
            type: 'string',
            description: 'Search query',
            required: true,
          },
        ],
      },
      {
        name: 'explain_code',
        description: 'Explain what a piece of NAVA code does',
        parameters: [
          {
            name: 'code',
            type: 'string',
            description: 'NAVA code to explain',
            required: true,
          },
        ],
      },
    ];
  }

  /**
   * Execute a tool call
   */
  async executeTool(toolName: string, arguments: Record<string, any>): Promise<NAVAToolResult> {
    switch (toolName) {
      case 'run_nava':
        return await this.runNAVA(arguments.code, arguments.interpreter);
      case 'compile_nava':
        return await this.compileNAVA(arguments.code);
      case 'read_file':
        return await this.readFile(arguments.path);
      case 'write_file':
        return await this.writeFile(arguments.path, arguments.content);
      case 'list_files':
        return await this.listFiles(arguments.path);
      case 'get_compiler_help':
        return await this.getCompilerHelp(arguments.error_code);
      case 'search_docs':
        return await this.searchDocs(arguments.query);
      case 'explain_code':
        return await this.explainCode(arguments.code);
      default:
        return {
          success: false,
          error: `Unknown tool: ${toolName}`,
        };
    }
  }

  /**
   * Run NAVA code
   */
  private async runNAVA(code: string, interpreter?: string): Promise<NAVAToolResult> {
    try {
      // Import the NAVA engine
      const { solveOptimalPath } = await import('../apps/univarm-starter/engine/nava-engine');
      
      // For now, extract path optimization parameters if present
      // In full implementation, this would parse and execute any NAVA code
      const startMatch = code.match(/start⋋\s*=\s*Vector3D⋋\(([^)]+)\)/);
      const goalMatch = code.match(/goal⋋\s*=\s*Vector3D⋋\(([^)]+)\)/);
      
      if (startMatch && goalMatch) {
        const startCoords = startMatch[1].split(',').map(s => parseFloat(s.trim()));
        const goalCoords = goalMatch[1].split(',').map(s => parseFloat(s.trim()));
        
        const result = await solveOptimalPath({
          start: { x: startCoords[0], y: startCoords[1], z: startCoords[2] || 0 },
          goal: { x: goalCoords[0], y: goalCoords[1], z: goalCoords[2] || 0 },
          interpreter: interpreter as any || 'auto',
        });
        
        return {
          success: true,
          output: `Path computed successfully:\nPoints: ${result.points.length}\nCost: ${result.cost}\nInterpreter: ${result.metadata?.interpreter}\nExecution time: ${result.metadata?.executionTime}ms`,
          metadata: result.metadata,
        };
      }
      
      return {
        success: true,
        output: 'NAVA code parsed. For full execution, please use the NAVA compiler/interpreter.',
      };
    } catch (error: any) {
      return {
        success: false,
        error: error.message || 'Failed to run NAVA code',
        exitCode: 1,
      };
    }
  }

  /**
   * Compile NAVA code (syntax check)
   */
  private async compileNAVA(code: string): Promise<NAVAToolResult> {
    try {
      // Basic syntax validation
      const errors: string[] = [];
      
      // Check for program declaration
      if (!code.includes('program') && !code.includes('program⋋')) {
        errors.push('Missing program declaration. Expected: program⋋ or program');
      }
      
      // Check for balanced brackets
      const openBrackets = (code.match(/\(/g) || []).length;
      const closeBrackets = (code.match(/\)/g) || []).length;
      if (openBrackets !== closeBrackets) {
        errors.push(`Unbalanced brackets: ${openBrackets} open, ${closeBrackets} close`);
      }
      
      // Check for NAVA operators
      if (!code.includes('⋋') && !code.includes('let') && !code.includes('Vector3D')) {
        errors.push('No NAVA operators detected. Is this valid NAVA code?');
      }
      
      if (errors.length > 0) {
        return {
          success: false,
          error: errors.join('\n'),
          exitCode: 1,
        };
      }
      
      return {
        success: true,
        output: 'NAVA code syntax appears valid. For full compilation, use the NAVA compiler.',
      };
    } catch (error: any) {
      return {
        success: false,
        error: error.message || 'Compilation check failed',
        exitCode: 1,
      };
    }
  }

  /**
   * Read a file
   */
  private async readFile(path: string): Promise<NAVAToolResult> {
    try {
      const { fileService } = await import('./file-service');
      const content = await fileService.readFile(path);
      return {
        success: true,
        output: content,
        metadata: { path },
      };
    } catch (error: any) {
      return {
        success: false,
        error: `Failed to read file: ${error.message}`,
      };
    }
  }

  /**
   * Write a file
   */
  private async writeFile(path: string, content: string): Promise<NAVAToolResult> {
    try {
      const { fileService } = await import('./file-service');
      await fileService.createFile(path, content);
      return {
        success: true,
        output: `File written successfully: ${path}`,
        metadata: { path, size: content.length },
      };
    } catch (error: any) {
      return {
        success: false,
        error: `Failed to write file: ${error.message}`,
      };
    }
  }

  /**
   * List files in directory
   */
  private async listFiles(path?: string): Promise<NAVAToolResult> {
    try {
      const { fileService } = await import('./file-service');
      const project = fileService.getCurrentProject();
      if (!project) {
        return {
          success: false,
          error: 'No project open',
        };
      }
      
      const files = project.files || [];
      const fileList = files.map(f => `${f.isDirectory ? '📁' : '📄'} ${f.path}`).join('\n');
      
      return {
        success: true,
        output: fileList || 'No files found',
        metadata: { count: files.length },
      };
    } catch (error: any) {
      return {
        success: false,
        error: `Failed to list files: ${error.message}`,
      };
    }
  }

  /**
   * Get compiler help
   */
  private async getCompilerHelp(errorCode: string): Promise<NAVAToolResult> {
    // This would query NAVA documentation
    const helpText = `NAVA Compiler Help for: ${errorCode}\n\nCommon errors:\n- Type mismatch: Check Vector3D dimensions\n- Undefined operator: Ensure ⋋ suffix is used\n- Missing program: Start with program⋋\n\nFor detailed help, see NAVA documentation.`;
    
    return {
      success: true,
      output: helpText,
    };
  }

  /**
   * Search documentation
   */
  private async searchDocs(query: string): Promise<NAVAToolResult> {
    // This would search embedded NAVA docs
    const results = `NAVA Documentation Search: "${query}"\n\nRelevant operators:\n- find_optimal_path⋋: Computes optimal paths on manifolds\n- Vector3D⋋: 3D vector type\n- navigation_field⋋: Defines navigation energy fields\n- DAAT contracts: deadline⋋, dmr⋋, jitter⋋\n\nFor full documentation, see the NAVA language specification.`;
    
    return {
      success: true,
      output: results,
    };
  }

  /**
   * Explain NAVA code
   */
  private async explainCode(code: string): Promise<NAVAToolResult> {
    // This would use the NAVA assistant to explain code
    const explanation = `Code Explanation:\n\nThis NAVA code:\n1. Defines navigation variables\n2. Uses VΛNC operators (⋋ suffix)\n3. Computes optimal paths\n4. Outputs results\n\nFor detailed explanation, ask the NAVA assistant.`;
    
    return {
      success: true,
      output: explanation,
    };
  }
}

export const navaToolsService = new NAVAToolsService();

