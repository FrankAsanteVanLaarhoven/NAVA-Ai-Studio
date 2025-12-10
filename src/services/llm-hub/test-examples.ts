/**
 * End-to-End Test Examples
 * 
 * These are concrete examples you can copy straight into your backend tests.
 * They demonstrate the complete flow from user prompt to tool execution to response.
 */

import type { ChatRequest, ChatResponse, ToolCall } from './types';

/**
 * Example 1: Intent → NAVA → Run
 * 
 * User wants to plan a path, AI generates NAVA code, then runs it.
 */
export const example1_IntentToNAVAToRun = {
  description: 'User asks for path planning, AI generates code and runs it',
  
  userMessage: 'Plan a smooth path from [0,0] to [5,5] on the plane, avoiding a circular obstacle at [2,2] radius 1.',
  
  expectedToolCalls: [
    {
      id: 'call_1',
      type: 'function' as const,
      function: {
        name: 'generate_nava_from_prompt',
        arguments: JSON.stringify({
          prompt: 'Plan a smooth path from [0,0] to [5,5] on R^2 avoiding a circular obstacle at [2,2] radius 1.',
          target_level: 'beginner',
        }),
      },
    },
  ],
  
  toolResult1: {
    code: `let M = euclidean_plane()
let start = [0.0, 0.0]
let goal  = [5.0, 5.0]

// Circular obstacle at (2,2) with radius 1
let obstacle = disc(center: [2.0, 2.0], radius: 1.0)

let field = navigation_field(
  manifold: M,
  start: start,
  goal: goal,
  obstacles: [obstacle],
  cost: geodesic_cost()
)

let path = compute_optimal_path(field)
visualize(path, field)`,
    explanation: 'Generated NAVA code for path planning with obstacle avoidance',
    target_level: 'beginner',
  },
  
  expectedSecondToolCall: {
    id: 'call_2',
    type: 'function' as const,
    function: {
      name: 'run_nava_program',
      arguments: JSON.stringify({
        code: `let M = euclidean_plane()
let start = [0.0, 0.0]
let goal  = [5.0, 5.0]
let obstacle = disc(center: [2.0, 2.0], radius: 1.0)
let field = navigation_field(manifold: M, start: start, goal: goal, obstacles: [obstacle], cost: geodesic_cost())
let path = compute_optimal_path(field)
visualize(path, field)`,
        max_steps: 256,
      }),
    },
  },
  
  toolResult2: {
    type: 'program_result',
    manifold: 'R2',
    paths: [
      {
        name: 'optimal',
        points: [[0, 0], [0.5, 0.3], [1.0, 0.8], [1.5, 1.5], [2.5, 2.5], [3.5, 3.5], [4.5, 4.5], [5, 5]],
      },
    ],
    obstacles: [
      { type: 'circle', center: { x: 2, y: 2 }, radius: 1 },
    ],
    cost_field: {
      grid: [[1.2, 1.1, 1.0], [1.1, 1.0, 0.9], [1.0, 0.9, 0.8]],
      x_coords: [0, 2.5, 5],
      y_coords: [0, 2.5, 5],
    },
    metrics: {
      length: 7.32,
      max_curvature: 0.43,
      max_jerk: 0.12,
    },
  },
  
  expectedFinalResponse: 'I\'ve generated NAVA code for planning a path from (0,0) to (5,5) while avoiding a circular obstacle at (2,2) with radius 1. The code defines a Euclidean plane, sets start and goal points, creates a disc obstacle, builds a navigation field, and computes the optimal path. I\'ve also run the program and the optimal path has length 7.32 units, successfully avoiding the obstacle.',
};

/**
 * Example 2: Explanation at GCSE Level
 * 
 * User asks for explanation, AI uses explain_nava_code tool.
 */
export const example2_ExplanationAtGCSE = {
  description: 'User asks for simple explanation, AI uses explain_nava_code tool',
  
  userMessage: 'Explain what this NAVA code is doing in simple terms.\n\nlet M = euclidean_plane()\nlet start = [0.0, 0.0]\nlet goal = [5.0, 5.0]\nlet field = navigation_field(manifold: M, start: start, goal: goal)\nlet path = compute_optimal_path(field)',
  
  expectedToolCall: {
    id: 'call_1',
    type: 'function' as const,
    function: {
      name: 'explain_nava_code',
      arguments: JSON.stringify({
        code: `let M = euclidean_plane()
let start = [0.0, 0.0]
let goal = [5.0, 5.0]
let field = navigation_field(manifold: M, start: start, goal: goal)
let path = compute_optimal_path(field)`,
        audience: 'gcse',
      }),
    },
  },
  
  toolResult: {
    explanation: `In plain terms:

We say "our world is a flat 2D plane" (euclidean_plane).

We mark a starting point at (0,0) and a goal at (5,5).

navigation_field builds an invisible "slope" that always points you roughly towards the goal.

compute_optimal_path then follows that slope from start to goal to get a smooth, efficient path.

It's like drawing the steepest downhill path on a hill that leads you from your house to your friend's house.`,
    audience: 'gcse',
  },
  
  expectedFinalResponse: `In plain terms:

We say "our world is a flat 2D plane" (euclidean_plane).

We mark a starting point at (0,0) and a goal at (5,5).

navigation_field builds an invisible "slope" that always points you roughly towards the goal.

compute_optimal_path then follows that slope from start to goal to get a smooth, efficient path.

It's like drawing the steepest downhill path on a hill that leads you from your house to your friend's house.`,
};

/**
 * Example 3: Optimization Request
 * 
 * User asks to optimize code, AI uses optimise_nava_expression tool.
 */
export const example3_Optimization = {
  description: 'User asks to optimize code, AI refactors it',
  
  userMessage: 'Can you make this code more readable?\n\nlet M=euclidean_plane()\nlet s=[0,0]\nlet g=[5,5]\nlet f=navigation_field(manifold:M,start:s,goal:g)\nlet p=compute_optimal_path(f)',
  
  expectedToolCall: {
    id: 'call_1',
    type: 'function' as const,
    function: {
      name: 'optimise_nava_expression',
      arguments: JSON.stringify({
        code: `let M=euclidean_plane()
let s=[0,0]
let g=[5,5]
let f=navigation_field(manifold:M,start:s,goal:g)
let p=compute_optimal_path(f)`,
        goal: 'readability',
      }),
    },
  },
  
  toolResult: {
    code: `let M = euclidean_plane()
let start = [0.0, 0.0]
let goal = [5.0, 5.0]
let field = navigation_field(manifold: M, start: start, goal: goal)
let path = compute_optimal_path(field)`,
    explanation: 'Refactored variable names from single letters to descriptive names (s → start, g → goal, f → field, p → path). Added proper spacing for readability.',
    goal: 'readability',
  },
  
  expectedFinalResponse: 'I\'ve refactored the code for better readability. The main changes:\n\n1. Renamed single-letter variables to descriptive names: `s` → `start`, `g` → `goal`, `f` → `field`, `p` → `path`\n2. Added proper spacing around operators and after colons\n3. Used consistent formatting\n\nThe semantics remain exactly the same - it still computes the same optimal path.',
};

/**
 * Example 4: Scenario Generation
 * 
 * User asks for scenario variants, AI uses generate_scenarios_from_code tool.
 */
export const example4_ScenarioGeneration = {
  description: 'User asks for scenario variants, AI generates multiple scenarios',
  
  userMessage: 'Generate 3 variants of this scenario with different start/goal positions:\n\nlet M = euclidean_plane()\nlet start = [0.0, 0.0]\nlet goal = [5.0, 5.0]\nlet field = navigation_field(manifold: M, start: start, goal: goal)\nlet path = compute_optimal_path(field)',
  
  expectedToolCall: {
    id: 'call_1',
    type: 'function' as const,
    function: {
      name: 'generate_scenarios_from_code',
      arguments: JSON.stringify({
        base_code: `let M = euclidean_plane()
let start = [0.0, 0.0]
let goal = [5.0, 5.0]
let field = navigation_field(manifold: M, start: start, goal: goal)
let path = compute_optimal_path(field)`,
        num_variants: 3,
        variation_type: 'starts_goals',
      }),
    },
  },
  
  toolResult: {
    scenarios: [
      {
        id: 'scenario-1',
        name: 'Scenario 1',
        code: `let M = euclidean_plane()
let start = [1.0, 1.0]
let goal = [6.0, 6.0]
let field = navigation_field(manifold: M, start: start, goal: goal)
let path = compute_optimal_path(field)`,
        manifold: 'R3',
        start: { x: 1.0, y: 1.0, z: 0 },
        goal: { x: 6.0, y: 6.0, z: 0 },
        obstacles: [],
      },
      {
        id: 'scenario-2',
        name: 'Scenario 2',
        code: `let M = euclidean_plane()
let start = [-2.0, -2.0]
let goal = [3.0, 3.0]
let field = navigation_field(manifold: M, start: start, goal: goal)
let path = compute_optimal_path(field)`,
        manifold: 'R3',
        start: { x: -2.0, y: -2.0, z: 0 },
        goal: { x: 3.0, y: 3.0, z: 0 },
        obstacles: [],
      },
      {
        id: 'scenario-3',
        name: 'Scenario 3',
        code: `let M = euclidean_plane()
let start = [2.0, 0.0]
let goal = [7.0, 5.0]
let field = navigation_field(manifold: M, start: start, goal: goal)
let path = compute_optimal_path(field)`,
        manifold: 'R3',
        start: { x: 2.0, y: 0.0, z: 0 },
        goal: { x: 7.0, y: 5.0, z: 0 },
        obstacles: [],
      },
    ],
    variation_type: 'starts_goals',
  },
  
  expectedFinalResponse: 'I\'ve generated 3 scenario variants with different start/goal positions:\n\n**Scenario 1**: start=(1,1), goal=(6,6)\n**Scenario 2**: start=(-2,-2), goal=(3,3)\n**Scenario 3**: start=(2,0), goal=(7,5)\n\nEach scenario uses the same navigation field structure but with different start and goal points. You can run each one to see how the path changes.',
};

/**
 * Example 5: Documentation Lookup
 * 
 * User asks about NAVA syntax, AI uses fetch_nava_docs tool.
 */
export const example5_DocumentationLookup = {
  description: 'User asks about NAVA syntax, AI looks up documentation',
  
  userMessage: 'How do I create a navigation field?',
  
  expectedToolCall: {
    id: 'call_1',
    type: 'function' as const,
    function: {
      name: 'fetch_nava_docs',
      arguments: JSON.stringify({
        query: 'navigation_field',
        max_results: 5,
      }),
    },
  },
  
  toolResult: {
    query: 'navigation_field',
    results: [
      {
        title: 'navigation_field()',
        content: 'Creates a navigation field on a manifold with start, goal, obstacles, and cost function.',
        example: 'let field = navigation_field(manifold: M, start: [0,0], goal: [5,5], obstacles: [], cost: geodesic_cost())',
      },
    ],
    count: 1,
  },
  
  expectedFinalResponse: 'To create a navigation field, use the `navigation_field()` function:\n\n```nava\nlet field = navigation_field(\n  manifold: M,\n  start: [0, 0],\n  goal: [5, 5],\n  obstacles: [],\n  cost: geodesic_cost()\n)\n```\n\nIt takes:\n- `manifold`: The manifold (e.g., `euclidean_plane()`)\n- `start`: Starting position as a vector\n- `goal`: Goal position as a vector\n- `obstacles`: Optional list of obstacles\n- `cost`: Cost function (e.g., `geodesic_cost()` for shortest path)',
};

/**
 * Helper function to create a test request
 */
export function createTestRequest(
  userMessage: string,
  tools: any[] = [],
  context?: { currentCode?: string; currentFile?: string }
): ChatRequest {
  return {
    messages: [
      {
        role: 'system',
        content: 'You are NAVA-Assistant...', // Would use actual system prompt
      },
      {
        role: 'user',
        content: context?.currentCode
          ? `Current code:\n\`\`\`nava\n${context.currentCode}\n\`\`\`\n\n${userMessage}`
          : userMessage,
      },
    ],
    tools: tools.length > 0 ? tools : undefined,
    toolChoice: tools.length > 0 ? 'auto' : 'none',
    temperature: 0.2,
    maxTokens: 2000,
  };
}

/**
 * Helper function to simulate tool execution
 */
export async function simulateToolExecution(
  toolCall: ToolCall,
  toolImplementations: Record<string, (args: any) => Promise<any>>
): Promise<{ role: 'tool'; name: string; content: string; tool_call_id: string }> {
  const args = typeof toolCall.function.arguments === 'string'
    ? JSON.parse(toolCall.function.arguments)
    : toolCall.function.arguments;
  
  const implementation = toolImplementations[toolCall.function.name];
  if (!implementation) {
    throw new Error(`Unknown tool: ${toolCall.function.name}`);
  }
  
  const result = await implementation(args);
  
  return {
    role: 'tool',
    name: toolCall.function.name,
    content: JSON.stringify(result),
    tool_call_id: toolCall.id,
  };
}

