#!/usr/bin/env python3
"""
DAAT/PDL/RT-shields Data Generator
Generates training examples for timing-aware NAVA code
"""

import json
import random
from typing import Dict, List

class DAATPDLDataGenerator:
    """Generate DAAT/PDL/RT-shields training examples"""
    
    def generate_daat_example(self) -> Dict:
        """Generate DAAT contract example"""
        frequency = random.choice([50, 100, 200])
        dmr = random.choice([0.001, 0.005, 0.01])  # 0.1%, 0.5%, 1%
        jitter = random.choice([5, 10, 20])  # ms
        
        base_code = """let M = euclidean_plane()
let start = [0.0, 0.0]
let goal = [5.0, 5.0]
let field = navigation_field(manifold: M, start: start, goal: goal, obstacles: [], cost: geodesic_cost())
let path = compute_optimal_path(field)"""
        
        base_lines = base_code.split('\n')
        indented_base = '\n'.join('      ' + line for line in base_lines)
        
        daat_code = f"""program daat_navigation⋋
  -- DAAT Contract: {frequency}Hz, DMR ≤ {dmr*100}%, Jitter ≤ {jitter}ms
  
  let navigation_operator⋋ = 
    deadline⋋(
      frequency⋋({frequency}),
      dmr⋋({dmr}),
      jitter⋋({jitter})
    ) ∘⋋
    (
{indented_base}
    )
  
  verify_daat_contract⋋(navigation_operator⋋, frequency⋋({frequency}))
end program"""
        
        instruction = f"Write NAVA code for a navigation problem that runs at {frequency} Hz with a deadline miss rate of at most {dmr*100}% and jitter of at most {jitter} ms. Include DAAT timing contracts."
        
        return {
            "instruction": instruction,
            "input": "",
            "output": daat_code,
        }
    
    def generate_pdl_example(self) -> Dict:
        """Generate PDL tier example"""
        base_code = """let M = euclidean_plane()
let start = [0.0, 0.0]
let goal = [5.0, 5.0]
let field = navigation_field(manifold: M, start: start, goal: goal, obstacles: [], cost: geodesic_cost())
let path = compute_optimal_path(field)"""
        
        base_lines = base_code.split('\n')
        indented_base = '\n'.join('      ' + line for line in base_lines)
        
        pdl_code = f"""program pdl_navigation⋋
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
{indented_base}
    ),
    time_budget⋋(10ms),
    chunk_size⋋(32)
  )
  
  -- Tier-2: Offline planner (can take longer)
  let tier2_planner⋋ = offline_planner⋋(
    (
{indented_base}
    ),
    optimization_horizon⋋(100)
  )
  
  -- Hierarchical composition
  let hierarchical_control⋋ = 
    tier2_planner⋋ 
    |> fallback_to⋋(tier1_vla⋋)
    |> emergency_to⋋(tier0_reflex⋋)
end program"""
        
        instruction = "Write NAVA code with a multi-tier PDL architecture: Tier-0 for reflex actions (<1ms), Tier-1 for chunked VLA calls (10ms budget), and Tier-2 for offline planning. Use hierarchical composition with fallbacks."
        
        return {
            "instruction": instruction,
            "input": "",
            "output": pdl_code,
        }
    
    def generate_rt_shields_example(self) -> Dict:
        """Generate RT-shields contract example"""
        base_code = """let M = euclidean_plane()
let start = [0.0, 0.0]
let goal = [5.0, 5.0]
let field = navigation_field(manifold: M, start: start, goal: goal, obstacles: [], cost: geodesic_cost())
let path = compute_optimal_path(field)"""
        
        base_lines = base_code.split('\n')
        indented_base = '\n'.join('    ' + line for line in base_lines)
        
        rt_shields_code = f"""program rt_shields_navigation⋋
  -- RT-shields: Real-time safety contracts
  
  let navigation_controller⋋ = (
{indented_base}
  )
  
  -- RT-shields contract
  let timing_contract⋋ = rt_shield_contract⋋(
    dmr_threshold⋋(0.005),      -- DMR ≤ 0.5%
    aj_p95_threshold⋋(10),      -- AJ p95 ≤ 10ms
    ttp_threshold⋋(10),         -- TTP ≤ 10ms
    ttb_threshold⋋(100)         -- TTB ≤ 100ms
  )
  
  -- Verify contract
  verify_rt_shield⋋(navigation_controller⋋, timing_contract⋋)
end program"""
        
        instruction = "Write NAVA code with RT-shields timing contracts: DMR ≤ 0.5%, arrival jitter (p95) ≤ 10ms, time-to-process ≤ 10ms, and time-to-budget ≤ 100ms. Include contract verification."
        
        return {
            "instruction": instruction,
            "input": "",
            "output": rt_shields_code,
        }
    
    def generate_batch(self, num_daat: int = 20, num_pdl: int = 20, num_rt_shields: int = 10) -> List[Dict]:
        """Generate batch of DAAT/PDL/RT-shields examples"""
        examples = []
        
        for _ in range(num_daat):
            examples.append(self.generate_daat_example())
        
        for _ in range(num_pdl):
            examples.append(self.generate_pdl_example())
        
        for _ in range(num_rt_shields):
            examples.append(self.generate_rt_shields_example())
        
        return examples

def main():
    generator = DAATPDLDataGenerator()
    
    print("🚀 Generating DAAT/PDL/RT-shields Training Data...")
    examples = generator.generate_batch(num_daat=20, num_pdl=20, num_rt_shields=10)
    
    print(f"✅ Generated {len(examples)} examples")
    print(f"   DAAT: 20 examples")
    print(f"   PDL: 20 examples")
    print(f"   RT-shields: 10 examples")
    
    # Save
    output_path = "../NAVA Studio IDE/data/nava_instruct_daat_pdl.jsonl"
    with open(output_path, 'w') as f:
        for ex in examples:
            f.write(json.dumps(ex) + '\n')
    
    print(f"💾 Saved to {output_path}")
    
    # Show sample
    print(f"\n📝 Sample DAAT example:")
    print(json.dumps(examples[0], indent=2))

if __name__ == "__main__":
    main()

