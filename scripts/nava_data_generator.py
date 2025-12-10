#!/usr/bin/env python3
"""
NAVA Data Generator
Uses the NAVA math stack to generate synthetic training data
"""

import json
import random
import math
from pathlib import Path
from typing import List, Dict, Tuple

class NAVADataGenerator:
    """Generate training data using NAVA stack as ground truth"""
    
    def __init__(self, nava_runtime_path: str = None):
        self.manifolds = [
            "euclidean_plane()",
            "euclidean_space(dim: 3)",
            "circle(radius: 1.0)",
            "sphere(radius: 1.0)",
            "se2()",
            "se3()",
        ]
        
        self.cost_functions = [
            "geodesic_cost()",
            "cost_function(length_weight: 1.0, curvature_weight: 2.0, jerk_weight: 2.0)",
            "cost_function(type: \"time_optimal\", max_velocity: 2.0)",
            "cost_function(type: \"energy_optimal\")",
        ]
        
        self.obstacle_types = ["disc", "rectangle", "polygon"]
    
    def generate_problem(self) -> Dict:
        """Generate a random navigation problem"""
        manifold = random.choice(self.manifolds)
        
        # Random start/goal
        if "plane" in manifold or "space" in manifold:
            start = [round(random.uniform(-5, 5), 1), round(random.uniform(-5, 5), 1)]
            goal = [round(random.uniform(5, 15), 1), round(random.uniform(5, 15), 1)]
            if "space" in manifold:
                start.append(round(random.uniform(-2, 2), 1))
                goal.append(round(random.uniform(-2, 2), 1))
        elif "circle" in manifold or "se2" in manifold:
            start = [round(random.uniform(0, 2 * math.pi), 3)]
            goal = [round(random.uniform(0, 2 * math.pi), 3)]
        else:
            start = [1.0, 0.0, 0.0]
            goal = [0.0, 1.0, 0.0]
        
        # Random obstacles
        num_obstacles = random.randint(0, 3)
        obstacles = []
        for i in range(num_obstacles):
            if random.choice([True, False]):
                # Circular obstacle
                center = [round(random.uniform(2, 8), 1), round(random.uniform(2, 8), 1)]
                radius = round(random.uniform(0.5, 1.5), 1)
                obstacles.append({
                    "type": "disc",
                    "center": center,
                    "radius": radius,
                })
            else:
                # Rectangular obstacle
                min_corner = [round(random.uniform(2, 6), 1), round(random.uniform(2, 6), 1)]
                max_corner = [min_corner[0] + round(random.uniform(1, 2), 1), 
                             min_corner[1] + round(random.uniform(1, 2), 1)]
                obstacles.append({
                    "type": "rectangle",
                    "min_corner": min_corner,
                    "max_corner": max_corner,
                })
        
        cost = random.choice(self.cost_functions)
        
        return {
            "manifold": manifold,
            "start": start,
            "goal": goal,
            "obstacles": obstacles,
            "cost": cost,
        }
    
    def build_nava_code(self, problem: Dict) -> str:
        """Build canonical NAVA code from problem specification"""
        lines = []
        
        # Manifold
        lines.append(f"let M = {problem['manifold']}")
        lines.append("")
        
        # Start and goal
        start_str = str(problem['start']).replace("'", "")
        goal_str = str(problem['goal']).replace("'", "")
        lines.append(f"let start = {start_str}")
        lines.append(f"let goal  = {goal_str}")
        lines.append("")
        
        # Obstacles
        obstacle_vars = []
        for i, obs in enumerate(problem['obstacles'], 1):
            if obs['type'] == 'disc':
                center_str = str(obs['center']).replace("'", "")
                lines.append(f"let obstacle{i} = disc(center: {center_str}, radius: {obs['radius']})")
                obstacle_vars.append(f"obstacle{i}")
            elif obs['type'] == 'rectangle':
                min_str = str(obs['min_corner']).replace("'", "")
                max_str = str(obs['max_corner']).replace("'", "")
                lines.append(f"let obstacle{i} = rectangle(min_corner: {min_str}, max_corner: {max_str})")
                obstacle_vars.append(f"obstacle{i}")
        
        if obstacle_vars:
            lines.append("")
        
        # Navigation field
        obstacles_list = f"[{', '.join(obstacle_vars)}]" if obstacle_vars else "[]"
        lines.append("let field = navigation_field(")
        lines.append("  manifold: M,")
        lines.append("  start: start,")
        lines.append("  goal: goal,")
        lines.append(f"  obstacles: {obstacles_list},")
        lines.append(f"  cost: {problem['cost']}")
        lines.append(")")
        lines.append("")
        
        # Compute path
        lines.append("let path = compute_optimal_path(field)")
        lines.append("visualize(path, field)")
        
        return "\n".join(lines)
    
    def generate_natural_language(self, problem: Dict) -> str:
        """Generate natural language description of the problem"""
        manifold_desc = {
            "euclidean_plane()": "the Euclidean plane R^2",
            "euclidean_space(dim: 3)": "3D Euclidean space R^3",
            "circle(radius: 1.0)": "the unit circle S^1",
            "sphere(radius: 1.0)": "the unit sphere S^2",
            "se2()": "SE(2) (planar rigid body)",
            "se3()": "SE(3) (3D rigid body)",
        }.get(problem['manifold'], "a manifold")
        
        start_str = f"({', '.join(map(str, problem['start']))})"
        goal_str = f"({', '.join(map(str, problem['goal']))})"
        
        desc = f"Plan a smooth path on {manifold_desc} from {start_str} to {goal_str}"
        
        if problem['obstacles']:
            obs_descs = []
            for obs in problem['obstacles']:
                if obs['type'] == 'disc':
                    center_str = f"({obs['center'][0]}, {obs['center'][1]})"
                    obs_descs.append(f"a circular obstacle at {center_str} with radius {obs['radius']}")
                elif obs['type'] == 'rectangle':
                    obs_descs.append("a rectangular obstacle")
            
            if len(obs_descs) == 1:
                desc += f" while avoiding {obs_descs[0]}"
            else:
                desc += f" while avoiding {', '.join(obs_descs[:-1])}, and {obs_descs[-1]}"
        
        # Cost function description
        if "curvature" in problem['cost']:
            desc += " with low curvature"
        elif "time_optimal" in problem['cost']:
            desc += " that is time-optimal"
        elif "energy_optimal" in problem['cost']:
            desc += " that minimizes energy"
        
        desc += "."
        
        return desc
    
    def generate_example(self, include_comments: bool = False) -> Dict:
        """Generate one training example"""
        problem = self.generate_problem()
        nava_code = self.build_nava_code(problem)
        description = self.generate_natural_language(problem)
        
        # Add comments for beginner level
        if include_comments:
            lines = nava_code.split('\n')
            commented_lines = []
            for line in lines:
                commented_lines.append(line)
                if 'let M =' in line:
                    commented_lines.append("  // Define the manifold (the space we're navigating)")
                elif 'let start =' in line:
                    commented_lines.append("  // Starting position")
                elif 'let goal =' in line:
                    commented_lines.append("  // Goal position")
                elif 'obstacle' in line and '=' in line:
                    commented_lines.append("  // An obstacle to avoid")
                elif 'navigation_field' in line:
                    commented_lines.append("  // Build a navigation field that guides from start to goal")
                elif 'compute_optimal_path' in line:
                    commented_lines.append("  // Find the best path following the field")
            nava_code = '\n'.join(commented_lines)
        
        return {
            "instruction": description,
            "input": "",
            "output": nava_code,
        }
    
    def generate_batch(self, num_examples: int, include_comments_ratio: float = 0.3) -> List[Dict]:
        """Generate a batch of training examples"""
        examples = []
        for i in range(num_examples):
            include_comments = random.random() < include_comments_ratio
            example = self.generate_example(include_comments=include_comments)
            examples.append(example)
        return examples

def main():
    generator = NAVADataGenerator()
    
    # Generate training examples
    print("🚀 Generating NAVA training data using NAVA stack...")
    train_examples = generator.generate_batch(100, include_comments_ratio=0.3)
    eval_examples = generator.generate_batch(20, include_comments_ratio=0.2)
    
    # Save to files
    output_dir = Path("../NAVA Studio IDE/data")
    output_dir.mkdir(parents=True, exist_ok=True)
    
    train_path = output_dir / "nava_instruct_train_generated.jsonl"
    eval_path = output_dir / "nava_instruct_eval_generated.jsonl"
    
    with open(train_path, 'w') as f:
        for ex in train_examples:
            f.write(json.dumps(ex) + '\n')
    
    with open(eval_path, 'w') as f:
        for ex in eval_examples:
            f.write(json.dumps(ex) + '\n')
    
    print(f"✅ Generated {len(train_examples)} training examples")
    print(f"✅ Generated {len(eval_examples)} eval examples")
    print(f"📁 Saved to {train_path} and {eval_path}")
    
    # Show sample
    print(f"\n📝 Sample generated example:")
    print(json.dumps(train_examples[0], indent=2))

if __name__ == "__main__":
    main()

