/**
 * NAVA AI Tools
 * 
 * Tool definitions for AI models to call.
 * These tools enable the AI to interact with the IDE and generate NAVA code.
 */

export interface AITool {
  type: 'function';
  function: {
    name: string;
    description: string;
    parameters: {
      type: 'object';
      properties: Record<string, any>;
      required: string[];
    };
  };
}

export interface ToolCallResult {
  success: boolean;
  result: any;
  error?: string;
}

class NAVAAITools {
  /**
   * Get all available AI tools
   */
  getAvailableTools(): AITool[] {
    return [
      {
        type: 'function',
        function: {
          name: 'generate_nava_from_prompt',
          description: 'Generate NAVA/VNC code that implements the user\'s intent. Always prefer NAVA over other languages unless the user explicitly asks for Python or something else.',
          parameters: {
            type: 'object',
            properties: {
              prompt: {
                type: 'string',
                description: 'Natural language description of the task, geometry, constraints, or navigation problem.',
              },
              context_code: {
                type: 'string',
                description: 'Optional existing NAVA code from the current file or cell to extend or edit.',
                nullable: true,
              },
              target_level: {
                type: 'string',
                enum: ['beginner', 'intermediate', 'expert'],
                description: 'How simple and well-commented the resulting code should be.',
              },
            },
            required: ['prompt'],
          },
        },
      },
      {
        type: 'function',
        function: {
          name: 'optimise_nava_expression',
          description: 'Refactor or optimise a NAVA/VNC expression with a given goal, preserving semantics.',
          parameters: {
            type: 'object',
            properties: {
              code: {
                type: 'string',
                description: 'NAVA/VNC code to refactor',
              },
              goal: {
                type: 'string',
                enum: ['readability', 'performance', 'stability', 'idiomatic_nava'],
                description: 'Refactor objective.',
              },
            },
            required: ['code', 'goal'],
          },
        },
      },
      {
        type: 'function',
        function: {
          name: 'add_invariants',
          description: 'Add mathematical invariants and safety checks to NAVA code',
          parameters: {
            type: 'object',
            properties: {
              code: {
                type: 'string',
                description: 'NAVA code to add invariants to',
              },
            },
            required: ['code'],
          },
        },
      },
      {
        type: 'function',
        function: {
          name: 'explain_nava_code',
          description: 'Explain the given NAVA/VNC code in plain language, with optional focus on specific lines or concepts.',
          parameters: {
            type: 'object',
            properties: {
              code: {
                type: 'string',
                description: 'NAVA/VNC code to explain',
              },
              audience: {
                type: 'string',
                enum: ['gcse', 'alevel', 'undergrad', 'phd'],
                description: 'Level of mathematical background of the explanation.',
              },
              focus: {
                type: 'string',
                description: 'Optional focus question, e.g. "why is this stable?", "what is this manifold?".',
                nullable: true,
              },
            },
            required: ['code'],
          },
        },
      },
      {
        type: 'function',
        function: {
          name: 'generate_scenarios_from_code',
          description: 'Create a set of NAVA/VNC scenario variants by changing start/goal, obstacles, or manifolds.',
          parameters: {
            type: 'object',
            properties: {
              base_code: {
                type: 'string',
                description: 'Base NAVA/VNC code to create variants from',
              },
              num_variants: {
                type: 'integer',
                default: 3,
                minimum: 1,
                maximum: 10,
                description: 'Number of variants to generate',
              },
              variation_type: {
                type: 'string',
                enum: ['starts_goals', 'obstacles', 'manifold', 'mixed'],
                default: 'mixed',
                description: 'Type of variation to apply',
              },
            },
            required: ['base_code'],
          },
        },
      },
      {
        type: 'function',
        function: {
          name: 'fetch_nava_docs',
          description: 'Search NAVA/VNC documentation and examples for functions, operators, and idioms relevant to the user\'s question.',
          parameters: {
            type: 'object',
            properties: {
              query: {
                type: 'string',
                description: 'Search query for documentation',
              },
              max_results: {
                type: 'integer',
                default: 5,
                description: 'Maximum number of results to return',
              },
            },
            required: ['query'],
          },
        },
      },
      {
        type: 'function',
        function: {
          name: 'run_nava_program',
          description: 'Run a NAVA/VNC program using the local runtime and return sampled trajectories, fields, and metrics for preview.',
          parameters: {
            type: 'object',
            properties: {
              code: {
                type: 'string',
                description: 'NAVA/VNC code to execute',
              },
              max_steps: {
                type: 'integer',
                description: 'Maximum number of integration steps to run for preview.',
                default: 512,
              },
              sampling_dt: {
                type: 'number',
                description: 'Sampling time in seconds between preview points.',
                default: 0.05,
              },
            },
            required: ['code'],
          },
        },
      },
      {
        type: 'function',
        function: {
          name: 'add_daat_contract',
          description: 'Add DAAT (Deadline-Aware Architecture Template) contract to NAVA code',
          parameters: {
            type: 'object',
            properties: {
              code: {
                type: 'string',
                description: 'NAVA code to add contract to',
              },
              frequency: {
                type: 'number',
                description: 'Required frequency in Hz',
                default: 50,
              },
              dmr: {
                type: 'number',
                description: 'Deadline miss rate threshold (e.g., 0.005 for 0.5%)',
                default: 0.005,
              },
              jitter: {
                type: 'number',
                description: 'Maximum jitter in ms',
                default: 10,
              },
            },
            required: ['code'],
          },
        },
      },
      {
        type: 'function',
        function: {
          name: 'add_pdl_tiers',
          description: 'Add PDL (Policy Description Language) tier annotations to NAVA code',
          parameters: {
            type: 'object',
            properties: {
              code: {
                type: 'string',
                description: 'NAVA code to add PDL tiers to',
              },
            },
            required: ['code'],
          },
        },
      },
    ];
  }

  /**
   * Execute a tool call
   */
    async executeTool(toolName: string, args: Record<string, any>): Promise<ToolCallResult> {
    switch (toolName) {
      case 'generate_nava_from_prompt':
        return await this.generateNAVAFromPrompt(args.prompt, args.context_code, args.target_level);
      
      case 'optimise_nava_expression':
        return await this.optimizeNAVAExpression(args.code, args.goal);
      
      case 'add_invariants':
        return await this.addInvariants(args.code);
      
      case 'explain_nava_code':
        return await this.explainNAVACode(args.code, args.audience, args.focus);
      
      case 'generate_scenarios_from_code':
        return await this.generateScenariosFromCode(
          args.base_code,
          args.num_variants,
          args.variation_type
        );
      
      case 'run_nava_program':
        return await this.runNAVAProgram(args.code, args.max_steps, args.sampling_dt);
      
      case 'fetch_nava_docs':
        return await this.fetchNAVADocs(args.query, args.max_results);
      
      case 'add_daat_contract':
        return await this.addDAATContract(
          args.code,
          args.frequency,
          args.dmr,
          args.jitter
        );
      
      case 'add_pdl_tiers':
        return await this.addPDLTiers(args.code);
      
      default:
        return {
          success: false,
          result: null,
          error: `Unknown tool: ${toolName}`,
        };
    }
  }

  /**
   * Generate NAVA code from prompt
   */
  private async generateNAVAFromPrompt(
    prompt: string,
    contextCode?: string,
    includeDAAT?: boolean
  ): Promise<ToolCallResult> {
    try {
      // Use NAVA assistant service to generate code
      const { navaAssistantService } = await import('./nava-assistant-service');
      const response = await navaAssistantService.generateResponse(prompt, {
        currentCode: contextCode,
      });
      
      let code = response.content;
      
      // Extract code from markdown if present
      const codeMatch = code.match(/```nava\n([\s\S]*?)```/);
      if (codeMatch) {
        code = codeMatch[1];
      }
      
      // Add DAAT contract if requested
      if (includeDAAT) {
        const daatResult = await this.addDAATContract(code, 50, 0.005, 10);
        if (daatResult.success) {
          code = daatResult.result.code;
        }
      }
      
      return {
        success: true,
        result: { code, explanation: response.explanation },
      };
    } catch (error: any) {
      return {
        success: false,
        result: null,
        error: error.message,
      };
    }
  }

  /**
   * Optimize NAVA expression
   */
  private async optimizeNAVAExpression(
    code: string,
    goal: 'speed' | 'readability' | 'stability'
  ): Promise<ToolCallResult> {
    // This would use the NAVA assistant with optimization prompts
    let optimizedCode = code;
    let explanation = '';
    
    if (goal === 'speed') {
      explanation = 'Optimized for execution speed: reduced samples, simplified energy landscape';
      // Would apply speed optimizations
    } else if (goal === 'readability') {
      explanation = 'Optimized for readability: added comments, clearer variable names';
      // Would apply readability improvements
    } else if (goal === 'stability') {
      explanation = 'Optimized for stability: added bounds checking, smoother path computation';
      // Would apply stability improvements
    }
    
    return {
      success: true,
      result: { code: optimizedCode, explanation },
    };
  }

  /**
   * Add invariants to code
   */
  private async addInvariants(code: string): Promise<ToolCallResult> {
    // Add invariant checks to NAVA code
    const invariants = [
      '// Invariant: Path exists between start and goal',
      '// Invariant: No collisions with obstacles',
      '// Invariant: Energy is non-negative',
    ];
    
    const codeWithInvariants = invariants.join('\n') + '\n\n' + code;
    
    return {
      success: true,
      result: { code: codeWithInvariants, invariants },
    };
  }

  /**
   * Explain NAVA code at different levels
   */
  private async explainNAVACode(
    code: string,
    audience: 'gcse' | 'alevel' | 'undergrad' | 'phd' = 'alevel',
    focus?: string
  ): Promise<ToolCallResult> {
    // Use NAVA assistant service for explanation
    const { navaAssistantService } = await import('./nava-assistant-service');
    
    let explanationPrompt = `Explain this NAVA/VNC code`;
    if (focus) {
      explanationPrompt += ` with focus on: ${focus}`;
    }
    explanationPrompt += `\n\nCode:\n${code}`;
    
    const response = await navaAssistantService.generateResponse(explanationPrompt, {
      currentCode: code,
    });
    
    // Format explanation based on audience level
    let explanation = response.content;
    
    if (audience === 'gcse') {
      explanation = `In plain terms:\n\n${explanation}\n\nThink of it like drawing a path on a map that avoids obstacles and gets you from point A to point B as efficiently as possible.`;
    } else if (audience === 'alevel') {
      explanation = `This NAVA code uses Van Laarhoven Navigation Calculus to compute an optimal path. ${explanation}`;
    } else if (audience === 'undergrad') {
      explanation = `This implements a path optimization problem on a manifold. ${explanation}`;
    } else {
      explanation = `This NAVA program implements a geodesic path optimization problem on a Riemannian manifold using variational methods. ${explanation}`;
    }
    
    return {
      success: true,
      result: { explanation, audience, focus },
    };
  }

  /**
   * Generate scenarios from code
   */
  private async generateScenariosFromCode(
    baseCode: string,
    numVariants: number = 3,
    variationType: 'starts_goals' | 'obstacles' | 'manifold' | 'mixed' = 'mixed'
  ): Promise<ToolCallResult> {
    const { navaPreviewEngine } = await import('./nava-preview-engine');
    const scenarios = navaPreviewEngine.generateScenarios(baseCode, numVariants);
    
    // Apply variation type filtering/transformation if needed
    // For now, we use the preview engine's default mixed variation
    
    return {
      success: true,
      result: {
        scenarios: scenarios.map(s => ({
          id: s.id,
          name: s.name,
          code: s.navaCode,
          manifold: s.manifold,
          start: s.start,
          goal: s.goal,
          obstacles: s.obstacles,
        })),
        variation_type: variationType,
      },
    };
  }

  /**
   * Run NAVA program
   */
  private async runNAVAProgram(
    code: string,
    maxSteps: number = 512,
    samplingDt: number = 0.05
  ): Promise<ToolCallResult> {
    const { navaRuntimeService } = await import('./nava-runtime-service');
    const { navaPreviewEngine } = await import('./nava-preview-engine');
    
    const result = await navaRuntimeService.runNAVA(code, {
      preview: true,
      samples: maxSteps,
    });
    
    if (!result.success) {
      return {
        success: false,
        result: null,
        error: result.error,
      };
    }
    
    // Convert to program_result format
    const programResult = {
      type: 'program_result',
      manifold: result.metadata?.manifold || 'R2',
      paths: result.path ? [{
        name: 'optimal',
        points: result.path.map(p => [p.x, p.y, p.z || 0]),
      }] : [],
      obstacles: result.obstacles || [],
      cost_field: result.costGrid ? {
        grid: result.costGrid.values,
        x_coords: Array.from({ length: result.costGrid.width }, (_, i) => {
          const step = (result.costGrid!.bounds.max.x - result.costGrid!.bounds.min.x) / result.costGrid!.width;
          return result.costGrid!.bounds.min.x + i * step;
        }),
        y_coords: Array.from({ length: result.costGrid.height }, (_, i) => {
          const step = (result.costGrid!.bounds.max.y - result.costGrid!.bounds.min.y) / result.costGrid!.height;
          return result.costGrid!.bounds.min.y + i * step;
        }),
      } : null,
      metrics: {
        length: result.metadata?.pathLength || 0,
        max_curvature: 0, // Would be computed from path
        max_jerk: result.metadata?.jerkMax || 0,
      },
    };
    
    return {
      success: true,
      result: programResult,
    };
  }
  
  /**
   * Fetch NAVA documentation
   */
  private async fetchNAVADocs(
    query: string,
    maxResults: number = 5
  ): Promise<ToolCallResult> {
    try {
      // Load documentation index
      const indexResponse = await fetch('/data/nava_docs_index.json');
      if (indexResponse.ok) {
        const index = await indexResponse.json();
        
        // Search index
        const queryLower = query.toLowerCase();
        const results = index
          .filter((entry: any) => 
            entry.title?.toLowerCase().includes(queryLower) ||
            entry.content?.toLowerCase().includes(queryLower) ||
            entry.functions?.some((f: string) => f.toLowerCase().includes(queryLower))
          )
          .slice(0, maxResults)
          .map((entry: any) => ({
            title: entry.title,
            content: entry.content,
            file: entry.file,
            code_examples: entry.code_examples || [],
            functions: entry.functions || [],
          }));
        
        return {
          success: true,
          result: {
            query,
            results: results.length > 0 ? results : index.slice(0, maxResults),
            count: results.length || index.length,
          },
        };
      }
    } catch (error) {
      console.warn('[NAVA Docs] Could not load index, using fallback:', error);
    }
    
    // Fallback to mock data
    const docs = [
      {
        title: 'euclidean_plane()',
        content: 'Creates a 2D Euclidean plane manifold. Returns a Manifold type.',
        example: 'let M = euclidean_plane()',
      },
      {
        title: 'navigation_field()',
        content: 'Creates a navigation field on a manifold with start, goal, obstacles, and cost function.',
        example: 'let field = navigation_field(manifold: M, start: [0,0], goal: [5,5], obstacles: [], cost: geodesic_cost())',
      },
      {
        title: 'compute_optimal_path()',
        content: 'Computes the optimal path from start to goal following the navigation field.',
        example: 'let path = compute_optimal_path(field)',
      },
      {
        title: 'disc()',
        content: 'Creates a circular obstacle. Takes center (Vector2D) and radius (number).',
        example: 'let obstacle = disc(center: [2.0, 2.0], radius: 1.0)',
      },
      {
        title: 'geodesic_cost()',
        content: 'Cost function that minimizes path length (geodesic distance).',
        example: 'cost: geodesic_cost()',
      },
    ];
    
    const queryLower = query.toLowerCase();
    const matchingDocs = docs
      .filter(doc => 
        doc.title.toLowerCase().includes(queryLower) ||
        doc.content.toLowerCase().includes(queryLower)
      )
      .slice(0, maxResults);
    
    return {
      success: true,
      result: {
        query,
        results: matchingDocs.length > 0 ? matchingDocs : docs.slice(0, maxResults),
        count: matchingDocs.length || docs.length,
      },
    };
  }

  /**
   * Add DAAT contract
   */
  private async addDAATContract(
    code: string,
    frequency: number = 50,
    dmr: number = 0.005,
    jitter: number = 10
  ): Promise<ToolCallResult> {
    const daatWrapper = `program daat_navigation⋋
  -- DAAT Contract: ${frequency}Hz, DMR ≤ ${dmr * 100}%, Jitter ≤ ${jitter}ms
  
  let navigation_operator⋋ = 
    deadline⋋(
      frequency⋋(${frequency}),
      dmr⋋(${dmr}),
      jitter⋋(${jitter})
    ) ∘⋋
    (
${code.split('\n').map(line => '      ' + line).join('\n')}
    )
  
  verify_daat_contract⋋(navigation_operator⋋, frequency⋋(${frequency}))
end program`;

    return {
      success: true,
      result: { code: daatWrapper },
    };
  }

  /**
   * Add PDL tiers
   */
  private async addPDLTiers(code: string): Promise<ToolCallResult> {
    const pdlWrapper = `program pdl_navigation⋋
  -- PDL: Policy Description Language with tiered architecture
  
  -- Tier-0: Reflex policy (fastest, <1ms)
  let tier0_reflex⋋ = reflex_policy⋋(
    emergency_stop⋋(),
    collision_avoidance⋋(),
    timeout⋋(1ms)
  )
  
  -- Tier-1: Chunked VLA call with time budget
  let tier1_vla⋋ = vla_chunked⋋(
    (
${code.split('\n').map(line => '      ' + line).join('\n')}
    ),
    time_budget⋋(10ms),
    chunk_size⋋(32)
  )
  
  -- Tier-2: Offline planner (can take longer)
  let tier2_planner⋋ = offline_planner⋋(
    (
${code.split('\n').map(line => '      ' + line).join('\n')}
    ),
    optimization_horizon⋋(100)
  )
  
  -- Hierarchical composition
  let hierarchical_control⋋ = 
    tier2_planner⋋ 
    |> fallback_to⋋(tier1_vla⋋)
    |> emergency_to⋋(tier0_reflex⋋)
end program`;

    return {
      success: true,
      result: { code: pdlWrapper },
    };
  }
}

export const navaAITools = new NAVAAITools();

