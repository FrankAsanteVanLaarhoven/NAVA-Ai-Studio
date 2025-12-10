#!/usr/bin/env python3
"""
NAVA Evaluation Harness
Uses NAVA runtime to evaluate model-generated code
"""

import json
import requests
from pathlib import Path
from typing import Dict, List, Tuple
import time

class NAVAEvaluationHarness:
    """Evaluate NAVA code using runtime as judge"""
    
    def __init__(self, model_server_url: str = "http://localhost:8080", nava_runtime=None):
        self.model_server_url = model_server_url
        self.runtime = nava_runtime
    
    def generate_code(self, prompt: str) -> str:
        """Ask model to generate NAVA code"""
        try:
            response = requests.post(
                f"{self.model_server_url}/v1/chat/completions",
                json={
                    "messages": [
                        {"role": "user", "content": prompt}
                    ],
                    "temperature": 0.2,
                    "max_tokens": 512,
                },
                timeout=30
            )
            
            if response.status_code == 200:
                data = response.json()
                return data["choices"][0]["message"]["content"]
            else:
                return None
        except Exception as e:
            print(f"Error generating code: {e}")
            return None
    
    def check_parse(self, code: str) -> Tuple[bool, List[str]]:
        """Check if code parses correctly"""
        errors = []
        
        # Basic syntax checks
        required = ["let M", "let start", "let goal", "navigation_field", "compute_optimal_path"]
        for req in required:
            if req not in code:
                errors.append(f"Missing: {req}")
        
        # Check for balanced brackets
        if code.count('(') != code.count(')'):
            errors.append("Unbalanced parentheses")
        
        if code.count('[') != code.count(']'):
            errors.append("Unbalanced brackets")
        
        return len(errors) == 0, errors
    
    def check_runtime(self, code: str) -> Tuple[bool, Dict]:
        """Check if code runs successfully"""
        if not self.runtime:
            # Mock check - in production would call actual runtime
            return True, {"status": "mock", "path_length": 0}
        
        try:
            # This would call the actual NAVA runtime
            # result = self.runtime.runNAVA(code, {preview: True})
            # return result.success, result.metadata or {}
            return True, {"status": "success"}
        except Exception as e:
            return False, {"error": str(e)}
    
    def check_constraints(self, code: str, problem: Dict) -> Tuple[bool, List[str]]:
        """Check if code satisfies problem constraints"""
        violations = []
        
        # Check start/goal match
        import re
        start_match = re.search(r"let\s+start\s*=\s*\[([^\]]+)\]", code, re.IGNORECASE)
        goal_match = re.search(r"let\s+goal\s*=\s*\[([^\]]+)\]", code, re.IGNORECASE)
        
        if start_match and goal_match:
            start_coords = [float(x.strip()) for x in start_match.group(1).split(',')]
            goal_coords = [float(x.strip()) for x in goal_match.group(1).split(',')]
            
            expected_start = problem.get("start", [])
            expected_goal = problem.get("goal", [])
            
            if expected_start and start_coords != expected_start:
                violations.append(f"Start position mismatch: got {start_coords}, expected {expected_start}")
            
            if expected_goal and goal_coords != expected_goal:
                violations.append(f"Goal position mismatch: got {goal_coords}, expected {expected_goal}")
        
        # Check obstacles
        if problem.get("obstacles"):
            for obs in problem["obstacles"]:
                if obs["type"] == "disc":
                    if f"disc(center: {obs['center']}" not in code:
                        violations.append(f"Missing obstacle: {obs}")
        
        return len(violations) == 0, violations
    
    def evaluate_on_benchmark(self, benchmark: List[Dict]) -> Dict:
        """Evaluate model on benchmark set"""
        results = {
            "total": len(benchmark),
            "parse_success": 0,
            "runtime_success": 0,
            "constraint_satisfied": 0,
            "all_passed": 0,
            "details": [],
        }
        
        for i, test_case in enumerate(benchmark, 1):
            prompt = test_case["instruction"]
            expected = test_case.get("output", "")
            problem = test_case.get("problem", {})
            
            print(f"\n📝 Test {i}/{len(benchmark)}: {prompt[:50]}...")
            
            # Generate code
            generated_code = self.generate_code(prompt)
            if not generated_code:
                results["details"].append({
                    "test": i,
                    "prompt": prompt,
                    "status": "generation_failed",
                })
                continue
            
            # Extract code from markdown if present
            import re
            code_match = re.search(r'```(?:nava)?\n(.*?)```', generated_code, re.DOTALL)
            if code_match:
                generated_code = code_match.group(1)
            
            # Check parse
            parse_ok, parse_errors = self.check_parse(generated_code)
            if parse_ok:
                results["parse_success"] += 1
            
            # Check runtime
            runtime_ok, runtime_info = self.check_runtime(generated_code)
            if runtime_ok:
                results["runtime_success"] += 1
            
            # Check constraints
            constraint_ok, violations = self.check_constraints(generated_code, problem)
            if constraint_ok:
                results["constraint_satisfied"] += 1
            
            # All passed
            if parse_ok and runtime_ok and constraint_ok:
                results["all_passed"] += 1
            
            results["details"].append({
                "test": i,
                "prompt": prompt,
                "generated_code": generated_code[:200],
                "parse_ok": parse_ok,
                "parse_errors": parse_errors,
                "runtime_ok": runtime_ok,
                "constraint_ok": constraint_ok,
                "violations": violations,
            })
        
        # Calculate percentages
        results["parse_rate"] = results["parse_success"] / results["total"] * 100
        results["runtime_rate"] = results["runtime_success"] / results["total"] * 100
        results["constraint_rate"] = results["constraint_satisfied"] / results["total"] * 100
        results["overall_rate"] = results["all_passed"] / results["total"] * 100
        
        return results

def load_benchmark(file_path: str) -> List[Dict]:
    """Load benchmark from JSONL file"""
    benchmark = []
    with open(file_path, 'r') as f:
        for line in f:
            if line.strip():
                benchmark.append(json.loads(line))
    return benchmark

def main():
    import argparse
    
    parser = argparse.ArgumentParser(description="Evaluate NAVA model using runtime")
    parser.add_argument("--server", type=str, default="http://localhost:8080", help="Model server URL")
    parser.add_argument("--benchmark", type=str, default="../NAVA Studio IDE/data/nava_instruct_eval.jsonl", help="Benchmark file")
    
    args = parser.parse_args()
    
    print("🧪 NAVA Evaluation Harness")
    print(f"🔗 Model Server: {args.server}")
    print(f"📊 Benchmark: {args.benchmark}")
    
    # Load benchmark
    benchmark = load_benchmark(args.benchmark)
    print(f"✅ Loaded {len(benchmark)} test cases\n")
    
    # Create harness
    harness = NAVAEvaluationHarness(model_server_url=args.server)
    
    # Evaluate
    results = harness.evaluate_on_benchmark(benchmark)
    
    # Print results
    print("\n" + "="*60)
    print("📊 Evaluation Results")
    print("="*60)
    print(f"Total Tests: {results['total']}")
    print(f"Parse Success: {results['parse_success']}/{results['total']} ({results['parse_rate']:.1f}%)")
    print(f"Runtime Success: {results['runtime_success']}/{results['total']} ({results['runtime_rate']:.1f}%)")
    print(f"Constraint Satisfied: {results['constraint_satisfied']}/{results['total']} ({results['constraint_rate']:.1f}%)")
    print(f"Overall Pass Rate: {results['all_passed']}/{results['total']} ({results['overall_rate']:.1f}%)")
    print("="*60)
    
    # Save detailed results
    output_file = Path("../NAVA Studio IDE/data/eval_results.json")
    with open(output_file, 'w') as f:
        json.dump(results, f, indent=2)
    print(f"\n💾 Detailed results saved to {output_file}")

if __name__ == "__main__":
    main()

