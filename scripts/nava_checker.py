#!/usr/bin/env python3
"""
NAVA Checker / Grader
Uses NAVA runtime to check if generated code is correct
"""

import json
import re
from typing import Dict, List, Tuple, Optional

class NAVAChecker:
    """Check NAVA code correctness using runtime"""
    
    def __init__(self, nava_runtime_service=None):
        self.runtime_service = nava_runtime_service
    
    def check_code(self, code: str) -> Dict:
        """
        Check if NAVA code is valid
        Returns: {ok: bool, errors: List[str], warnings: List[str]}
        """
        errors = []
        warnings = []
        
        # Basic syntax checks
        if not code.strip():
            errors.append("Code is empty")
            return {"ok": False, "errors": errors, "warnings": warnings}
        
        # Check for required components
        required_patterns = [
            (r"let\s+M\s*=\s*", "Manifold definition missing"),
            (r"let\s+start\s*=", "Start position missing"),
            (r"let\s+goal\s*=", "Goal position missing"),
            (r"navigation_field\s*\(", "Navigation field missing"),
            (r"compute_optimal_path\s*\(", "Path computation missing"),
        ]
        
        for pattern, error_msg in required_patterns:
            if not re.search(pattern, code, re.IGNORECASE):
                errors.append(error_msg)
        
        # Check for common mistakes
        if "navigation_field" in code and "manifold:" not in code:
            warnings.append("Navigation field should specify manifold explicitly")
        
        if "obstacles:" in code and "[]" in code and "obstacle" in code.lower():
            warnings.append("Obstacles defined but not used in field")
        
        # Try to parse and validate structure
        try:
            # Extract key values
            start_match = re.search(r"let\s+start\s*=\s*\[([^\]]+)\]", code, re.IGNORECASE)
            goal_match = re.search(r"let\s+goal\s*=\s*\[([^\]]+)\]", code, re.IGNORECASE)
            
            if start_match and goal_match:
                start_coords = [float(x.strip()) for x in start_match.group(1).split(',')]
                goal_coords = [float(x.strip()) for x in goal_match.group(1).split(',')]
                
                if len(start_coords) != len(goal_coords):
                    errors.append(f"Start and goal dimensions don't match: {len(start_coords)} vs {len(goal_coords)}")
                
                # Check if start == goal
                if all(abs(s - g) < 0.01 for s, g in zip(start_coords, goal_coords)):
                    warnings.append("Start and goal are very close (may cause issues)")
        except Exception as e:
            warnings.append(f"Could not parse coordinates: {e}")
        
        # If we have runtime service, try to execute
        if self.runtime_service:
            try:
                # This would call the actual NAVA runtime
                # For now, we do basic validation
                pass
            except Exception as e:
                errors.append(f"Runtime error: {e}")
        
        ok = len(errors) == 0
        
        return {
            "ok": ok,
            "errors": errors,
            "warnings": warnings,
        }
    
    def generate_correction(self, original_code: str, errors: List[str]) -> Optional[str]:
        """Generate corrected code based on errors"""
        corrected = original_code
        
        # Fix common errors
        if "Manifold definition missing" in errors:
            # Try to infer from context
            if "plane" in original_code.lower() or "R^2" in original_code:
                corrected = "let M = euclidean_plane()\n\n" + corrected
            elif "space" in original_code.lower() or "R^3" in original_code:
                corrected = "let M = euclidean_space(dim: 3)\n\n" + corrected
        
        if "Start position missing" in errors:
            # Add default start
            if "let M" in corrected:
                corrected = corrected.replace("let M", "let start = [0.0, 0.0]\nlet goal = [5.0, 5.0]\n\nlet M")
        
        if "Navigation field missing" in errors:
            # Add navigation field
            if "let goal" in corrected:
                corrected += "\n\nlet field = navigation_field(\n  manifold: M,\n  start: start,\n  goal: goal,\n  obstacles: [],\n  cost: geodesic_cost()\n)\n\nlet path = compute_optimal_path(field)\nvisualize(path, field)"
        
        return corrected if corrected != original_code else None
    
    def create_correction_example(self, prompt: str, wrong_code: str, errors: List[str]) -> Dict:
        """Create a training example for code correction"""
        corrected_code = self.generate_correction(wrong_code, errors)
        
        if not corrected_code:
            return None
        
        explanation = f"The original code had the following issues: {', '.join(errors)}. "
        explanation += "The corrected version fixes these by adding missing components and ensuring proper NAVA syntax."
        
        return {
            "instruction": f"Fix this NAVA code. {prompt}",
            "input": wrong_code,
            "output": f"{corrected_code}\n\n// Explanation: {explanation}",
        }

def main():
    checker = NAVAChecker()
    
    # Test with sample code
    test_codes = [
        # Good code
        """let M = euclidean_plane()
let start = [0.0, 0.0]
let goal = [5.0, 5.0]
let field = navigation_field(manifold: M, start: start, goal: goal, obstacles: [], cost: geodesic_cost())
let path = compute_optimal_path(field)""",
        
        # Bad code (missing manifold)
        """let start = [0.0, 0.0]
let goal = [5.0, 5.0]
let path = compute_optimal_path(field)""",
        
        # Bad code (missing field)
        """let M = euclidean_plane()
let start = [0.0, 0.0]
let goal = [5.0, 5.0]""",
    ]
    
    print("🧪 Testing NAVA Checker...\n")
    
    for i, code in enumerate(test_codes, 1):
        print(f"Test {i}:")
        print(f"Code:\n{code}\n")
        result = checker.check_code(code)
        print(f"✅ OK: {result['ok']}")
        if result['errors']:
            print(f"❌ Errors: {result['errors']}")
        if result['warnings']:
            print(f"⚠️  Warnings: {result['warnings']}")
        
        if not result['ok']:
            correction = checker.generate_correction(code, result['errors'])
            if correction:
                print(f"\n🔧 Corrected code:\n{correction}\n")
        
        print("-" * 60)

if __name__ == "__main__":
    main()

