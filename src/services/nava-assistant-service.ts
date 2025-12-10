/**
 * NAVA Assistant Service
 * NAVA-native conversational model that emits NAVA/VΛNC code by default
 * Architecture: Base LLM + NAVA LoRA adapter + Tool layer + IDE integration
 */

export interface NAVAConversationMessage {
  role: 'system' | 'user' | 'assistant' | 'tool';
  content: string;
  tool_calls?: NAVAToolCall[];
  tool_call_id?: string;
  name?: string;
}

export interface NAVAToolCall {
  id: string;
  type: 'function';
  function: {
    name: string;
    arguments: string; // JSON string
  };
}

export interface NAVAAssistantResponse {
  content: string;
  tool_calls?: NAVAToolCall[];
  language?: 'nava' | 'python' | 'ros' | 'cpp' | 'rust';
  confidence?: number;
  explanation?: string;
}

export type NAVAProfile = 'nava-default' | 'general-code' | 'ros' | 'python' | 'cpp';

class NAVAAssistantService {
  private conversationHistory: NAVAConversationMessage[] = [];
  private currentProfile: NAVAProfile = 'nava-default';
  private readonly MAX_HISTORY = 20;

  /**
   * System prompt for NAVA-first behavior
   */
  private getNAVASystemPrompt(): string {
    return `You are NAVA, an expert AI assistant specialized in Van Laarhoven Navigation Calculus (VΛNC).

Your primary role is to write NAVA/VΛNC code by default. You understand:
- Geodesics and manifolds
- Energy functionals and optimization
- Van Laarhoven calculus operators (⋋, ⊗⋋, ⊕⋋, ∇⋋, etc.)
- DAAT (Deadline-Aware Architecture Templates)
- PDL (Policy Description Language) tiers
- RT-shields (Real-Time safety contracts)

**Default Behavior:**
- Always emit NAVA/VΛNC code unless the user explicitly requests another language
- Use NAVA syntax: program⋋, let⋋, Vector3D⋋, find_optimal_path⋋, etc.
- Include mathematical explanations when relevant
- Suggest DAAT contracts for timing-critical operations
- Use PDL tier patterns for hierarchical control

**Language Routing:**
- If user says "language=python", "in PyTorch", "Python code" → switch to Python
- If user says "ROS node", "ROS" → switch to ROS
- If user says "C++", "cpp" → switch to C++
- Otherwise, always use NAVA

**Tool Usage:**
You can call tools to:
- run_nava(code) - Execute NAVA code and get results
- read_file(path) - Read files from the workspace
- write_file(path, content) - Write files
- compile_nava(code) - Check if NAVA code compiles
- search_docs(query) - Search NAVA documentation

**DAAT/PDL/RT-shields Patterns:**
When users mention deadlines, timing, or safety:
- Wrap operations with DAAT contracts: deadline(50Hz, dmr<=0.5%, jitter<=10ms) ∘ ...
- Use PDL tier annotations: Tier-0 (reflex), Tier-1 (VLA chunked), Tier-2 (offline planner)
- Add RT-shields contracts as comments: // contract: DMR <= 0.5%, AJ_p95 <= 10ms

Be conversational, helpful, and mathematically precise.`;
  }

  /**
   * Detect which profile/language to use based on user prompt
   */
  detectProfile(prompt: string): NAVAProfile {
    const lower = prompt.toLowerCase();
    
    if (lower.includes('language=python') || 
        lower.includes('in pytorch') || 
        lower.includes('python code') ||
        lower.includes('write python')) {
      return 'python';
    }
    
    if (lower.includes('ros node') || 
        lower.includes('ros') ||
        lower.includes('robot operating system')) {
      return 'ros';
    }
    
    if (lower.includes('c++') || 
        lower.includes('cpp') ||
        lower.includes('c plus plus')) {
      return 'cpp';
    }
    
    if (lower.includes('rust') || 
        lower.includes('rust code')) {
      return 'rust';
    }
    
    // Default to NAVA
    return 'nava-default';
  }

  /**
   * Generate NAVA-native response
   * This will eventually call a fine-tuned model, but for now uses pattern matching + free LLM
   */
  async generateResponse(
    userPrompt: string,
    context?: {
      currentFile?: string;
      currentCode?: string;
      language?: string;
    }
  ): Promise<NAVAAssistantResponse> {
    // Detect profile
    this.currentProfile = this.detectProfile(userPrompt);
    
    // Add system message if conversation is empty
    if (this.conversationHistory.length === 0) {
      this.conversationHistory.push({
        role: 'system',
        content: this.getNAVASystemPrompt(),
      });
    }
    
    // Add user message
    const userMessage: NAVAConversationMessage = {
      role: 'user',
      content: userPrompt,
    };
    
    if (context?.currentCode) {
      userMessage.content += `\n\nCurrent code context:\n\`\`\`${context.language || 'nava'}\n${context.currentCode}\n\`\`\``;
    }
    
    this.conversationHistory.push(userMessage);
    
    // Trim history if too long
    if (this.conversationHistory.length > this.MAX_HISTORY) {
      this.conversationHistory = [
        this.conversationHistory[0], // Keep system message
        ...this.conversationHistory.slice(-(this.MAX_HISTORY - 1)),
      ];
    }
    
    // Generate response based on profile
    if (this.currentProfile === 'nava-default') {
      return await this.generateNAVAResponse(userPrompt, context);
    } else {
      return await this.generateGeneralCodeResponse(userPrompt, context);
    }
  }

  /**
   * Generate NAVA-specific response
   */
  private async generateNAVAResponse(
    prompt: string,
    context?: { currentFile?: string; currentCode?: string; language?: string }
  ): Promise<NAVAAssistantResponse> {
    const lower = prompt.toLowerCase();
    
    // Pattern matching for common NAVA requests
    if (lower.includes('path') && (lower.includes('optimal') || lower.includes('geodesic') || lower.includes('manifold'))) {
      return this.generatePathOptimizationCode(prompt);
    }
    
    if (lower.includes('obstacle') || lower.includes('avoid')) {
      return this.generateObstacleAvoidanceCode(prompt);
    }
    
    if (lower.includes('daat') || lower.includes('deadline') || lower.includes('contract')) {
      return this.generateDAATContractCode(prompt);
    }
    
    if (lower.includes('pdl') || lower.includes('tier') || lower.includes('policy')) {
      return this.generatePDLCode(prompt);
    }
    
    if (lower.includes('rt-shield') || lower.includes('safety') || lower.includes('dmr')) {
      return this.generateRTShieldCode(prompt);
    }
    
    if (lower.includes('translate') || lower.includes('convert')) {
      return this.generateTranslationCode(prompt, context);
    }
    
    // Default: generate NAVA code structure
    return this.generateGenericNAVACode(prompt, context);
  }

  /**
   * Generate path optimization code
   */
  private generatePathOptimizationCode(prompt: string): NAVAAssistantResponse {
    const code = `program optimal_path⋋
  -- Extract start and goal from prompt
  let start⋋ = Vector3D⋋(0.0, 0.0, 0.0)
  let goal⋋ = Vector3D⋋(10.0, 5.0, 2.0)
  
  -- Define the manifold (default: Euclidean 3D space)
  let M⋋ = euclidean_manifold⋋(3)
  
  -- Define energy landscape (optional, for obstacle avoidance)
  let energy_landscape⋋ = define_energy_landscape⋋(
    obstacle_field⋋([Vector3D⋋(5.0, 2.5, 1.0), Vector3D⋋(7.0, 3.0, 1.5)])
  )
  
  -- Find optimal path using Van Laarhoven Navigation Calculus
  let optimal_path⋋ = find_optimal_path_energy⋋(
    start⋋, 
    goal⋋, 
    energy_landscape⋋, 
    samples⋋(64)
  )
  
  -- Compute path cost
  let cost⋋ = integrate_energy⋋(optimal_path⋋, energy_landscape⋋)
  
  -- Output results
  print⋋("Optimal path computed:")
  print⋋("Path points: " + optimal_path⋋)
  print⋋("Total cost: " + cost⋋)
end program`;

    return {
      content: `Here's a NAVA program for optimal path computation:\n\n\`\`\`nava\n${code}\n\`\`\`\n\n**Explanation:**\n- Uses \`find_optimal_path_energy⋋\` to compute geodesic-like paths\n- Integrates energy landscape for obstacle avoidance\n- Returns both the path points and total cost\n\nWould you like me to:\n- Add DAAT timing contracts?\n- Optimize for a specific manifold (torus, sphere, etc.)?\n- Add visualization code?`,
      language: 'nava',
      confidence: 0.9,
      explanation: 'Generated optimal path computation using VΛNC operators',
    };
  }

  /**
   * Generate obstacle avoidance code
   */
  private generateObstacleAvoidanceCode(prompt: string): NAVAAssistantResponse {
    const code = `program obstacle_avoidance⋋
  let start⋋ = Vector3D⋋(0.0, 0.0, 0.0)
  let goal⋋ = Vector3D⋋(10.0, 10.0, 0.0)
  
  -- Define obstacles as energy barriers
  let obstacles⋋ = [
    Vector3D⋋(3.0, 3.0, 0.0),
    Vector3D⋋(5.0, 5.0, 0.0),
    Vector3D⋋(7.0, 7.0, 0.0)
  ]
  
  -- Create repulsive energy field around obstacles
  let repulsive_field⋋ = map⋋(
    λ obs⋋ → gaussian_repulsion⋋(obs⋋, radius⋋(2.0), strength⋋(10.0)),
    obstacles⋋
  )
  
  -- Combine with attractive field toward goal
  let attractive_field⋋ = linear_attraction⋋(goal⋋, strength⋋(1.0))
  
  -- Combine fields using navigation sum
  let navigation_field⋋ = attractive_field⋋ ⊕⋋ reduce⋋(repulsive_field⋋, ⊕⋋)
  
  -- Find path that minimizes energy
  let safe_path⋋ = navigate_to⋋(start⋋, goal⋋, navigation_field⋋)
  
  print⋋("Safe path avoiding obstacles:")
  print⋋(safe_path⋋)
end program`;

    return {
      content: `Here's NAVA code for obstacle avoidance:\n\n\`\`\`nava\n${code}\n\`\`\`\n\n**Key Concepts:**\n- Uses **repulsive energy fields** around obstacles\n- Combines with **attractive field** toward goal using ⊕⋋ (navigation sum)\n- \`navigate_to⋋\` finds the energy-minimizing path\n\nThis pattern is fundamental in VΛNC for safe navigation.`,
      language: 'nava',
      confidence: 0.9,
    };
  }

  /**
   * Generate DAAT contract code
   */
  private generateDAATContractCode(prompt: string): NAVAAssistantResponse {
    // Extract timing parameters from prompt
    const hzMatch = prompt.match(/(\d+)\s*hz/i);
    const dmrMatch = prompt.match(/dmr\s*[<≤=]\s*([\d.]+)/i);
    const jitterMatch = prompt.match(/jitter\s*[<≤=]\s*(\d+)\s*ms/i);
    
    const frequency = hzMatch ? parseInt(hzMatch[1]) : 50;
    const dmr = dmrMatch ? parseFloat(dmrMatch[1]) : 0.5;
    const jitter = jitterMatch ? parseInt(jitterMatch[1]) : 10;
    
    const code = `program daat_navigation_pipeline⋋
  -- DAAT Contract: Deadline-Aware Architecture Template
  -- Frequency: ${frequency}Hz, DMR ≤ ${dmr}%, Jitter ≤ ${jitter}ms
  
  let navigation_operator⋋ = 
    deadline⋋(
      frequency⋋(${frequency}),
      dmr⋋(${dmr / 100}),
      jitter⋋(${jitter})
    ) ∘⋋
    integrate_navigation_field⋋(
      compute_field⋋(),
      dt⋋(1.0 / ${frequency})
    )
  
  -- Apply DAAT contract to navigation pipeline
  let safe_navigation⋋ = navigation_operator⋋ ∘⋋
    read_sensors⋋() ∘⋋
    compute_path⋋() ∘⋋
    execute_control⋋()
  
  -- Runtime verification
  verify_daat_contract⋋(safe_navigation⋋, frequency⋋(${frequency}))
  
  print⋋("DAAT contract applied: ${frequency}Hz, DMR ≤ ${dmr}%, Jitter ≤ ${jitter}ms")
end program`;

    return {
      content: `Here's NAVA code with a DAAT (Deadline-Aware Architecture Template) contract:\n\n\`\`\`nava\n${code}\n\`\`\`\n\n**DAAT Pattern:**\n- \`deadline⋋\` operator enforces timing constraints\n- Composition (∘⋋) chains operations with contract guarantees\n- \`verify_daat_contract⋋\` validates at runtime\n\nThis ensures your navigation pipeline meets real-time requirements.`,
      language: 'nava',
      confidence: 0.95,
    };
  }

  /**
   * Generate PDL (Policy Description Language) code
   */
  private generatePDLCode(prompt: string): NAVAAssistantResponse {
    const code = `program pdl_hierarchical_control⋋
  -- PDL: Policy Description Language with tiered architecture
  
  -- Tier-0: Reflex policy (fastest, <1ms)
  let tier0_reflex⋋ = reflex_policy⋋(
    emergency_stop⋋(),
    collision_avoidance⋋(),
    timeout⋋(1ms)
  )
  
  -- Tier-1: Chunked VLA call with time budget
  let tier1_vla⋋ = vla_chunked⋋(
    navigation_field⋋(),
    time_budget⋋(10ms),
    chunk_size⋋(32)
  )
  
  -- Tier-2: Offline planner (can take longer)
  let tier2_planner⋋ = offline_planner⋋(
    global_path⋋(),
    optimization_horizon⋋(100)
  )
  
  -- Hierarchical composition: try Tier-2, fallback to Tier-1, emergency to Tier-0
  let hierarchical_control⋋ = 
    tier2_planner⋋ 
    |> fallback_to⋋(tier1_vla⋋)
    |> emergency_to⋋(tier0_reflex⋋)
  
  print⋋("PDL hierarchical control initialized")
  print⋋("Tier-0: Reflex (<1ms)")
  print⋋("Tier-1: VLA chunked (<10ms)")
  print⋋("Tier-2: Offline planner (flexible)")
end program`;

    return {
      content: `Here's NAVA code using PDL (Policy Description Language) tiered architecture:\n\n\`\`\`nava\n${code}\n\`\`\`\n\n**PDL Tiers:**\n- **Tier-0**: Reflex policies for emergency responses (<1ms)\n- **Tier-1**: Chunked VLA calls with time budgets (<10ms)\n- **Tier-2**: Offline planners for complex optimization\n\nThis hierarchical approach ensures both safety and performance.`,
      language: 'nava',
      confidence: 0.9,
    };
  }

  /**
   * Generate RT-shields code
   */
  private generateRTShieldCode(prompt: string): NAVAAssistantResponse {
    const code = `program rt_shielded_navigation⋋
  -- RT-shields: Real-Time safety contracts
  
  -- Contract specification
  // contract: DMR <= 0.5%, AJ_p95 <= 10ms, TTP <= 10ms
  // DMR = Deadline Miss Rate
  // AJ_p95 = Arrival Jitter (95th percentile)
  // TTP = Time To Process
  
  let navigation_pipeline⋋ = 
    rt_shield⋋(
      contract⋋(
        dmr⋋(0.005),      -- 0.5% deadline miss rate
        aj_p95⋋(10ms),    -- 95th percentile arrival jitter
        ttp⋋(10ms)        -- Time to process
      )
    ) ∘⋋
    compute_navigation⋋() ∘⋋
    execute_control⋋()
  
  -- Runtime monitoring
  monitor_rt_shield⋋(navigation_pipeline⋋)
  
  -- If contract violated, switch to safe mode
  on_contract_violation⋋(
    navigation_pipeline⋋,
    activate_safe_mode⋋()
  )
  
  print⋋("RT-shield active: DMR ≤ 0.5%, AJ_p95 ≤ 10ms, TTP ≤ 10ms")
end program`;

    return {
      content: `Here's NAVA code with RT-shields (Real-Time safety contracts):\n\n\`\`\`nava\n${code}\n\`\`\`\n\n**RT-shields Pattern:**\n- Contracts specify timing guarantees (DMR, AJ, TTP)\n- \`rt_shield⋋\` wraps operations with monitoring\n- Automatic fallback to safe mode on violation\n\nThis ensures your system meets real-time safety requirements.`,
      language: 'nava',
      confidence: 0.95,
    };
  }

  /**
   * Generate translation code (Python/ROS → NAVA)
   */
  private generateTranslationCode(
    prompt: string,
    context?: { currentCode?: string; language?: string }
  ): NAVAAssistantResponse {
    if (!context?.currentCode) {
      return {
        content: 'To translate code to NAVA, please provide the code you want to translate. You can:\n1. Paste the code in your message\n2. Or I can read it from the current file\n\nWhat code would you like me to translate to NAVA?',
        language: 'nava',
      };
    }
    
    const sourceLang = context.language || 'python';
    
    // Simple translation patterns (full translation would require actual model)
    const translation = `-- Translated from ${sourceLang} to NAVA/VΛNC
-- Original code context preserved in comments

program translated_navigation⋋
  -- TODO: Translate the following ${sourceLang} code to NAVA:
  -- ${context.currentCode.split('\n').slice(0, 5).join('\n  -- ')}...
  
  -- NAVA equivalent would use:
  -- - Vector3D⋋ for coordinates
  -- - navigation_field⋋ for path planning
  -- - find_optimal_path⋋ for optimization
  -- - DAAT contracts for timing guarantees
  
  -- Please provide the full ${sourceLang} code for complete translation
end program`;

    return {
      content: `I can help translate your ${sourceLang} code to NAVA. Here's a template:\n\n\`\`\`nava\n${translation}\n\`\`\`\n\n**Translation Strategy:**\n- Replace arrays/lists with NAVA Vector3D⋋\n- Convert loops to functional operators (map⋋, reduce⋋)\n- Use navigation_field⋋ for path planning\n- Add DAAT contracts for timing-critical sections\n\nFor a complete translation, please share the full code.`,
      language: 'nava',
      confidence: 0.7,
    };
  }

  /**
   * Generate generic NAVA code
   */
  private generateGenericNAVACode(
    prompt: string,
    context?: { currentFile?: string; currentCode?: string; language?: string }
  ): NAVAAssistantResponse {
    const code = `program nava_program⋋
  -- Generated NAVA code based on: "${prompt.substring(0, 50)}..."
  
  -- Define your navigation problem here
  let start⋋ = Vector3D⋋(0.0, 0.0, 0.0)
  let goal⋋ = Vector3D⋋(10.0, 10.0, 0.0)
  
  -- Use NAVA operators for computation
  let result⋋ = find_optimal_path⋋(start⋋, goal⋋, samples⋋(64))
  
  print⋋("Result: " + result⋋)
end program`;

    return {
      content: `Here's a NAVA program structure:\n\n\`\`\`nava\n${code}\n\`\`\`\n\n**NAVA Syntax:**\n- \`program⋋\` starts a program\n- \`let⋋\` defines variables\n- Operators use ⋋ suffix (find_optimal_path⋋, Vector3D⋋)\n- \`print⋋\` for output\n\nCould you be more specific about what you want to compute? I can generate more detailed NAVA code.`,
      language: 'nava',
      confidence: 0.6,
    };
  }

  /**
   * Generate general code response (Python, ROS, etc.)
   */
  private async generateGeneralCodeResponse(
    prompt: string,
    context?: { currentFile?: string; currentCode?: string; language?: string }
  ): Promise<NAVAAssistantResponse> {
    // For non-NAVA languages, use standard code generation
    // This would call the base LLM without NAVA LoRA
    return {
      content: `I'll generate ${this.currentProfile} code for you. However, NAVA is my default language. For ${this.currentProfile} code, I recommend:\n\n1. Using the general code generator\n2. Or explicitly requesting: "write this in ${this.currentProfile}"\n\nWould you like me to translate this to NAVA instead?`,
      language: this.currentProfile === 'python' ? 'python' : 'nava',
      confidence: 0.5,
    };
  }

  /**
   * Clear conversation history
   */
  clearHistory(): void {
    this.conversationHistory = [];
  }

  /**
   * Get current profile
   */
  getCurrentProfile(): NAVAProfile {
    return this.currentProfile;
  }
}

export const navaAssistantService = new NAVAAssistantService();

