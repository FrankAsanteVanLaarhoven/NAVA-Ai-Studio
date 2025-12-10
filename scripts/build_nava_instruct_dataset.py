#!/usr/bin/env python3
"""
Build NAVA-Instruct Dataset Using NAVA Stack
Combines hand-written examples with NAVA-generated data
"""

import json
from pathlib import Path
from nava_data_generator import NAVADataGenerator
from nava_checker import NAVAChecker
from nava_explanation_generator import NAVAExplanationGenerator

def load_handwritten_examples(file_path: str):
    """Load hand-written examples"""
    examples = []
    with open(file_path, 'r') as f:
        for line in f:
            if line.strip():
                examples.append(json.loads(line))
    return examples

def main():
    print("🚀 Building NAVA-Instruct Dataset Using NAVA Stack")
    print("="*60)
    
    # Load hand-written seed examples
    seed_train = load_handwritten_examples("../NAVA Studio IDE/data/nava_instruct_train.jsonl")
    seed_eval = load_handwritten_examples("../NAVA Studio IDE/data/nava_instruct_eval.jsonl")
    
    print(f"📝 Loaded {len(seed_train)} hand-written training examples")
    print(f"📝 Loaded {len(seed_eval)} hand-written eval examples")
    
    # Initialize generators
    data_generator = NAVADataGenerator()
    checker = NAVAChecker()
    explanation_generator = NAVAExplanationGenerator()
    
    # Generate synthetic prompt→NAVA examples
    print("\n🔄 Generating synthetic prompt→NAVA examples...")
    synthetic_train = data_generator.generate_batch(100, include_comments_ratio=0.3)
    synthetic_eval = data_generator.generate_batch(20, include_comments_ratio=0.2)
    
    print(f"✅ Generated {len(synthetic_train)} synthetic training examples")
    print(f"✅ Generated {len(synthetic_eval)} synthetic eval examples")
    
    # Generate code→explanation examples from synthetic code
    print("\n🔄 Generating code→explanation examples...")
    all_code_examples = seed_train + synthetic_train
    explanation_examples = explanation_generator.generate_explanation_examples(all_code_examples)
    
    # Split explanations into train/eval (80/20)
    explanation_train = explanation_examples[:int(len(explanation_examples) * 0.8)]
    explanation_eval = explanation_examples[int(len(explanation_examples) * 0.8):]
    
    print(f"✅ Generated {len(explanation_train)} explanation training examples")
    print(f"✅ Generated {len(explanation_eval)} explanation eval examples")
    
    # Generate correction examples (bad→good)
    print("\n🔄 Generating correction examples...")
    correction_examples = []
    
    # Create some intentionally wrong code examples
    wrong_code_samples = [
        {
            "prompt": "Plan a path from (0,0) to (5,5)",
            "code": "let start = [0.0, 0.0]\nlet goal = [5.0, 5.0]\nlet path = compute_optimal_path(field)",
            "errors": ["Manifold definition missing", "Navigation field missing"],
        },
        {
            "prompt": "Navigate from (0,0) to (10,0) avoiding obstacles",
            "code": "let M = euclidean_plane()\nlet start = [0.0, 0.0]\nlet goal = [10.0, 0.0]",
            "errors": ["Navigation field missing", "Path computation missing"],
        },
    ]
    
    for sample in wrong_code_samples:
        correction = checker.create_correction_example(
            sample["prompt"],
            sample["code"],
            sample["errors"]
        )
        if correction:
            correction_examples.append(correction)
    
    print(f"✅ Generated {len(correction_examples)} correction examples")
    
    # Combine all examples
    print("\n🔄 Combining all examples...")
    
    # Training set: 70% prompt→NAVA, 30% explanation/correction
    train_prompt_nava = seed_train + synthetic_train
    train_explanations = explanation_train
    train_corrections = correction_examples
    
    # Mix them (70/30 ratio)
    total_train = len(train_prompt_nava) + len(train_explanations) + len(train_corrections)
    target_prompt_nava = int(total_train * 0.7)
    target_other = total_train - target_prompt_nava
    
    # Take all prompt→NAVA, sample from explanations/corrections
    final_train = train_prompt_nava[:target_prompt_nava]
    remaining_slots = target_prompt_nava - len(final_train)
    if remaining_slots > 0:
        final_train.extend(train_explanations[:remaining_slots])
        remaining_slots -= len(train_explanations[:remaining_slots])
        if remaining_slots > 0:
            final_train.extend(train_corrections[:remaining_slots])
    
    # Eval set: similar mix
    final_eval = seed_eval + synthetic_eval[:20] + explanation_eval[:10]
    
    print(f"✅ Final training set: {len(final_train)} examples")
    print(f"✅ Final eval set: {len(final_eval)} examples")
    
    # Save combined dataset
    output_dir = Path("../NAVA Studio IDE/data")
    output_dir.mkdir(parents=True, exist_ok=True)
    
    train_path = output_dir / "nava_instruct_train_enhanced.jsonl"
    eval_path = output_dir / "nava_instruct_eval_enhanced.jsonl"
    
    with open(train_path, 'w') as f:
        for ex in final_train:
            f.write(json.dumps(ex) + '\n')
    
    with open(eval_path, 'w') as f:
        for ex in final_eval:
            f.write(json.dumps(ex) + '\n')
    
    print(f"\n💾 Saved enhanced dataset:")
    print(f"   Training: {train_path}")
    print(f"   Eval: {eval_path}")
    
    # Statistics
    print("\n📊 Dataset Statistics:")
    print(f"   Prompt→NAVA: {len(train_prompt_nava)} examples")
    print(f"   Code→Explanation: {len(train_explanations)} examples")
    print(f"   Corrections: {len(train_corrections)} examples")
    print(f"   Total Training: {len(final_train)} examples")
    print(f"   Total Eval: {len(final_eval)} examples")

if __name__ == "__main__":
    main()

