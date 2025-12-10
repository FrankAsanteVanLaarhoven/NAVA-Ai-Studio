#!/usr/bin/env python3
"""
NAVA Explanation Generator
Uses NAVA stack knowledge to generate explanations at different levels
"""

import json
import re
from typing import Dict, List

class NAVAExplanationGenerator:
    """Generate explanations using NAVA stack structure knowledge"""
    
    def __init__(self):
        self.concept_templates = {
            "manifold": {
                "gcse": "the space or surface we're navigating on (like a flat map or a curved surface)",
                "alevel": "the configuration space or manifold where navigation occurs (e.g., R^2 for the plane, S^1 for a circle)",
                "undergrad": "the Riemannian manifold that defines the state space, with its metric determining distances",
                "phd": "the differentiable manifold M with metric tensor g, defining the configuration space and geodesic structure",
            },
            "navigation_field": {
                "gcse": "an invisible force field that points towards the goal and pushes away from obstacles",
                "alevel": "a vector field on the manifold that encodes the navigation policy, combining attractive forces to the goal and repulsive forces from obstacles",
                "undergrad": "a potential field or vector field on the manifold whose gradient guides optimal trajectories",
                "phd": "a smooth vector field X: M → TM that encodes the navigation policy, typically derived from a potential function V: M → R with X = -∇V",
            },
            "obstacle": {
                "gcse": "something the path must avoid, like a wall or barrier",
                "alevel": "a region of the configuration space that is forbidden, creating repulsive forces in the navigation field",
                "undergrad": "a closed subset O ⊂ M that defines forbidden regions, typically handled via repulsive potential terms",
                "phd": "a closed subset O ⊂ M with smooth boundary ∂O, incorporated into the navigation field via repulsive potential V_rep: M → R with V_rep → ∞ as x → ∂O",
            },
            "cost_function": {
                "gcse": "a rule that decides what makes a path 'good' (short, smooth, fast, etc.)",
                "alevel": "a function that assigns a cost to each possible path, with the optimal path minimizing this cost",
                "undergrad": "a functional J: C([0,T], M) → R that maps paths to real numbers, with optimal paths being minimizers of J",
                "phd": "a functional J[γ] = ∫₀ᵀ L(γ(t), γ'(t), t) dt where L is the Lagrangian, and optimal paths satisfy the Euler-Lagrange equations",
            },
        }
    
    def extract_concepts(self, code: str) -> List[str]:
        """Extract NAVA concepts from code"""
        concepts = []
        
        if re.search(r"euclidean_plane|R\^2", code, re.IGNORECASE):
            concepts.append("manifold")
        if "navigation_field" in code:
            concepts.append("navigation_field")
        if "obstacle" in code.lower() or "disc" in code or "rectangle" in code:
            concepts.append("obstacle")
        if "cost" in code.lower() or "geodesic" in code:
            concepts.append("cost_function")
        
        return concepts
    
    def explain_line(self, line: str, level: str) -> str:
        """Explain a single line of NAVA code"""
        line = line.strip()
        
        if "euclidean_plane" in line:
            return self.concept_templates["manifold"][level]
        elif "navigation_field" in line:
            return self.concept_templates["navigation_field"][level]
        elif "obstacle" in line.lower() or "disc" in line or "rectangle" in line:
            return self.concept_templates["obstacle"][level]
        elif "cost" in line.lower():
            return self.concept_templates["cost_function"][level]
        elif "compute_optimal_path" in line:
            if level == "gcse":
                return "finds the best path that follows the navigation field"
            elif level == "alevel":
                return "computes a trajectory that minimizes the cost functional subject to the navigation field"
            elif level == "undergrad":
                return "solves the optimal control problem to find a path γ* that minimizes J[γ]"
            else:
                return "solves the variational problem δJ[γ] = 0 to find the geodesic or optimal trajectory"
        
        return ""
    
    def generate_explanation(self, code: str, level: str = "alevel", focus: str = None) -> str:
        """Generate explanation at specified level"""
        lines = [l.strip() for l in code.split('\n') if l.strip() and not l.strip().startswith('//')]
        concepts = self.extract_concepts(code)
        
        explanation_parts = []
        
        if level == "gcse":
            explanation_parts.append("This code finds a path from one point to another on a flat surface.")
            if "obstacle" in code.lower():
                explanation_parts.append("There are obstacles in the way that the path must avoid.")
            explanation_parts.append("The navigation field is like an invisible guide that points towards the goal.")
            explanation_parts.append("The code then finds the best path that follows this guide.")
        elif level == "alevel":
            explanation_parts.append("This NAVA code sets up a navigation problem on a manifold.")
            if concepts:
                for concept in concepts:
                    if concept in self.concept_templates:
                        explanation_parts.append(f"- {self.concept_templates[concept][level]}")
            explanation_parts.append("The code computes an optimal path that minimizes the cost while following the navigation field.")
        elif level == "undergrad":
            explanation_parts.append("This implements a path planning problem on a Riemannian manifold.")
            explanation_parts.append("The navigation field encodes a potential whose gradient guides trajectories.")
            explanation_parts.append("The optimal path is found by solving the variational problem that minimizes the action functional.")
        else:  # phd
            explanation_parts.append("This NAVA program formulates an optimal control problem on a differentiable manifold M.")
            explanation_parts.append("The navigation field X: M → TM is derived from a potential V: M → R.")
            explanation_parts.append("The optimal trajectory γ*: [0,T] → M satisfies the Euler-Lagrange equations for the Lagrangian L.")
        
        if focus:
            explanation_parts.append(f"\nFocus: {focus}")
        
        return "\n\n".join(explanation_parts)
    
    def generate_explanation_examples(self, code_examples: List[Dict]) -> List[Dict]:
        """Generate explanation training examples from code examples"""
        explanation_examples = []
        
        levels = ["gcse", "alevel", "undergrad", "phd"]
        
        for code_example in code_examples:
            code = code_example.get("output", "")
            if not code:
                continue
            
            for level in levels:
                explanation = self.generate_explanation(code, level)
                
                example = {
                    "instruction": f"Explain this NAVA code at {level} level.",
                    "input": code,
                    "output": explanation,
                }
                explanation_examples.append(example)
        
        return explanation_examples

def main():
    generator = NAVAExplanationGenerator()
    
    # Sample code
    sample_code = """let M = euclidean_plane()
let start = [0.0, 0.0]
let goal = [5.0, 5.0]
let obstacle = disc(center: [2.0, 2.0], radius: 1.0)
let field = navigation_field(manifold: M, start: start, goal: goal, obstacles: [obstacle], cost: geodesic_cost())
let path = compute_optimal_path(field)"""
    
    print("📚 Generating NAVA Explanations...\n")
    
    for level in ["gcse", "alevel", "undergrad", "phd"]:
        print(f"{'='*60}")
        print(f"Level: {level.upper()}")
        print(f"{'='*60}")
        explanation = generator.generate_explanation(sample_code, level)
        print(explanation)
        print()

if __name__ == "__main__":
    main()

