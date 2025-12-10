/**
 * NAVA-Instruct Dataset Generator
 * 
 * Generates additional training examples following the same pattern.
 * Use this to expand the dataset from ~20 examples to 50-200.
 */

const fs = require('fs');
const path = require('path');

/**
 * Generate prompt → NAVA examples
 */
function generatePromptToNAVAExamples() {
  const examples = [];
  
  // Different manifolds
  const manifolds = [
    { name: 'euclidean_plane()', desc: 'Euclidean plane R^2' },
    { name: 'euclidean_space(dim: 3)', desc: '3D Euclidean space R^3' },
    { name: 'circle(radius: 1.0)', desc: 'unit circle S^1' },
    { name: 'sphere(radius: 1.0)', desc: 'unit sphere S^2' },
    { name: 'se2()', desc: 'SE(2) planar rigid body' },
    { name: 'se3()', desc: 'SE(3) 3D rigid body' },
  ];
  
  // Different obstacle configurations
  const obstacleConfigs = [
    { count: 0, desc: 'no obstacles' },
    { count: 1, desc: 'one circular obstacle' },
    { count: 2, desc: 'two circular obstacles' },
    { count: 3, desc: 'three obstacles in a pattern' },
  ];
  
  // Different cost functions
  const costFunctions = [
    { name: 'geodesic_cost()', desc: 'shortest path' },
    { name: 'cost_function(length_weight: 1.0, curvature_weight: 2.0)', desc: 'low curvature' },
    { name: 'cost_function(type: "time_optimal", max_velocity: 2.0)', desc: 'time-optimal' },
    { name: 'cost_function(type: "energy_optimal")', desc: 'energy-optimal' },
  ];
  
  // Generate combinations
  for (const manifold of manifolds.slice(0, 3)) { // Limit for demo
    for (const obstacle of obstacleConfigs.slice(0, 2)) {
      for (const cost of costFunctions.slice(0, 2)) {
        const start = [0.0, 0.0];
        const goal = [5.0, 5.0];
        
        let obstacleCode = '';
        if (obstacle.count > 0) {
          obstacleCode = `\nlet obstacle = disc(center: [2.0, 2.0], radius: 1.0)\n`;
        }
        
        const instruction = `Write NAVA code for a navigation problem on ${manifold.desc} from ${JSON.stringify(start)} to ${JSON.stringify(goal)} with ${obstacle.desc} using ${cost.desc} cost.`;
        
        let output = `let M = ${manifold.name}\n\nlet start = ${JSON.stringify(start)}\nlet goal  = ${JSON.stringify(goal)}\n${obstacleCode}\nlet field = navigation_field(\n  manifold: M,\n  start: start,\n  goal: goal,\n  obstacles: ${obstacle.count > 0 ? '[obstacle]' : '[]'},\n  cost: ${cost.name}\n)\n\nlet path = compute_optimal_path(field)\nvisualize(path, field)\n`;
        
        examples.push({
          instruction,
          input: '',
          output,
        });
      }
    }
  }
  
  return examples;
}

/**
 * Generate code → explanation examples
 */
function generateCodeToExplanationExamples() {
  const examples = [];
  
  const codeSnippets = [
    {
      code: `let M = euclidean_plane()
let start = [0.0, 0.0]
let goal  = [5.0, 5.0]
let field = navigation_field(manifold: M, start: start, goal: goal)
let path = compute_optimal_path(field)`,
      levels: ['gcse', 'alevel'],
    },
    {
      code: `let M = euclidean_plane()
let obstacle = disc(center: [3.0, 3.0], radius: 1.5)
let field = navigation_field(manifold: M, start: [0.0, 0.0], goal: [6.0, 6.0], obstacles: [obstacle])
let path = compute_optimal_path(field)`,
      levels: ['gcse', 'alevel', 'undergrad'],
    },
    {
      code: `let M = se2()
let start = [0.0, 0.0, 0.0]
let goal = [5.0, 5.0, 1.570796]
let cost = cost_function(length_weight: 1.0, curvature_weight: 2.0)
let field = navigation_field(manifold: M, start: start, goal: goal, obstacles: [], cost: cost)
let path = compute_optimal_path(field)`,
      levels: ['undergrad', 'phd'],
    },
  ];
  
  for (const snippet of codeSnippets) {
    for (const level of snippet.levels) {
      const instruction = `Explain this NAVA code at ${level} level.`;
      
      let output = '';
      if (level === 'gcse') {
        output = 'This code finds a path from one point to another on a flat surface. It creates a navigation field that guides the path and then finds the best route.';
      } else if (level === 'alevel') {
        output = 'This code sets up a navigation problem on the Euclidean plane. It defines start and goal points, creates a navigation field that points towards the goal, and computes an optimal path following that field.';
      } else if (level === 'undergrad') {
        output = 'This code formulates a path planning problem on a manifold. The navigation field encodes the cost structure, and compute_optimal_path finds a trajectory that minimises the integrated cost.';
      } else {
        output = 'This code solves an optimal control problem on a configuration manifold. The navigation field represents a potential whose gradient encodes the cost structure, and the optimal path is the geodesic minimising the action functional.';
      }
      
      examples.push({
        instruction,
        input: snippet.code,
        output,
      });
    }
  }
  
  return examples;
}

/**
 * Main function to generate and save dataset
 */
function generateDataset() {
  console.log('Generating NAVA-Instruct dataset...');
  
  const promptToNAVA = generatePromptToNAVAExamples();
  const codeToExplanation = generateCodeToExplanationExamples();
  
  // Combine and shuffle
  const allExamples = [...promptToNAVA, ...codeToExplanation];
  
  // Split 80/20
  const trainSize = Math.floor(allExamples.length * 0.8);
  const trainExamples = allExamples.slice(0, trainSize);
  const evalExamples = allExamples.slice(trainSize);
  
  // Write to files
  const dataDir = path.join(__dirname, '..', 'data');
  if (!fs.existsSync(dataDir)) {
    fs.mkdirSync(dataDir, { recursive: true });
  }
  
  // Append to existing files (or create new)
  const trainPath = path.join(dataDir, 'nava_instruct_train.jsonl');
  const evalPath = path.join(dataDir, 'nava_instruct_eval.jsonl');
  
  // Write train examples
  const trainLines = trainExamples.map(ex => JSON.stringify(ex)).join('\n');
  fs.appendFileSync(trainPath, '\n' + trainLines);
  
  // Write eval examples
  const evalLines = evalExamples.map(ex => JSON.stringify(ex)).join('\n');
  fs.appendFileSync(evalPath, '\n' + evalLines);
  
  console.log(`✅ Generated ${trainExamples.length} training examples`);
  console.log(`✅ Generated ${evalExamples.length} eval examples`);
  console.log(`📁 Saved to ${trainPath} and ${evalPath}`);
}

if (require.main === module) {
  generateDataset();
}

module.exports = { generateDataset, generatePromptToNAVAExamples, generateCodeToExplanationExamples };

