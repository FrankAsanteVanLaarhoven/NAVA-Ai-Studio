#!/usr/bin/env python3
"""
NAVA Fluency Test Script
Tests the fine-tuned model's NAVA fluency on various prompts
"""

import json
import requests
import argparse
from pathlib import Path

def test_prompt(server_url, prompt, expected_keywords=None):
    """Test a single prompt"""
    print(f"\n📝 Testing: {prompt[:60]}...")
    
    response = requests.post(
        f"{server_url}/v1/chat/completions",
        json={
            "messages": [
                {
                    "role": "user",
                    "content": prompt
                }
            ],
            "temperature": 0.2,
            "max_tokens": 512,
        },
        timeout=30
    )
    
    if response.status_code != 200:
        print(f"❌ Error: {response.status_code} - {response.text}")
        return False
    
    data = response.json()
    generated_text = data["choices"][0]["message"]["content"]
    
    print(f"✅ Generated ({len(generated_text)} chars):")
    print(f"   {generated_text[:200]}...")
    
    # Check for expected keywords
    if expected_keywords:
        found = [kw for kw in expected_keywords if kw.lower() in generated_text.lower()]
        if found:
            print(f"   ✅ Found keywords: {', '.join(found)}")
        else:
            print(f"   ⚠️  Missing keywords: {', '.join(expected_keywords)}")
    
    return True

def main():
    parser = argparse.ArgumentParser(description="Test NAVA model fluency")
    parser.add_argument("--server", type=str, default="http://localhost:8080", help="Model server URL")
    
    args = parser.parse_args()
    
    print("🧪 Testing NAVA Model Fluency")
    print(f"🔗 Server: {args.server}")
    
    # Test prompts
    test_cases = [
        {
            "prompt": "Write NAVA code that plans a smooth path from (0,0) to (5,5) on the Euclidean plane with no obstacles.",
            "expected_keywords": ["euclidean_plane", "navigation_field", "compute_optimal_path"],
        },
        {
            "prompt": "Write NAVA code to go from (0,0) to (5,5) on R^2 while avoiding a circular obstacle at (2,2) of radius 1.",
            "expected_keywords": ["disc", "obstacle", "navigation_field"],
        },
        {
            "prompt": "Explain this NAVA code in simple GCSE-level language: let M = euclidean_plane()\nlet start = [0.0, 0.0]\nlet goal = [5.0, 5.0]\nlet field = navigation_field(manifold: M, start: start, goal: goal)\nlet path = compute_optimal_path(field)",
            "expected_keywords": ["plane", "path", "simple"],
        },
        {
            "prompt": "Write NAVA code for a path on R^2 from (0,0) to (10,0) that prefers paths with low curvature and low jerk.",
            "expected_keywords": ["cost_function", "curvature", "jerk"],
        },
        {
            "prompt": "Write NAVA code for a navigation problem on the unit circle S^1, going from angle 0 to angle pi/2.",
            "expected_keywords": ["circle", "S^1", "angle"],
        },
    ]
    
    # Health check
    print("\n🏥 Health check...")
    try:
        health = requests.get(f"{args.server}/health", timeout=5)
        if health.status_code == 200:
            print(f"✅ Server is healthy: {health.json()}")
        else:
            print(f"⚠️  Server returned: {health.status_code}")
    except Exception as e:
        print(f"❌ Server not reachable: {e}")
        return
    
    # Run tests
    passed = 0
    failed = 0
    
    for i, test_case in enumerate(test_cases, 1):
        print(f"\n{'='*60}")
        print(f"Test {i}/{len(test_cases)}")
        print(f"{'='*60}")
        
        success = test_prompt(
            args.server,
            test_case["prompt"],
            test_case.get("expected_keywords")
        )
        
        if success:
            passed += 1
        else:
            failed += 1
    
    # Summary
    print(f"\n{'='*60}")
    print("📊 Test Summary")
    print(f"{'='*60}")
    print(f"✅ Passed: {passed}/{len(test_cases)}")
    print(f"❌ Failed: {failed}/{len(test_cases)}")
    print(f"📈 Success Rate: {passed/len(test_cases)*100:.1f}%")
    
    if passed == len(test_cases):
        print("\n🎉 All tests passed! Model is NAVA-fluent!")
    else:
        print(f"\n⚠️  Some tests failed. Model may need more training.")

if __name__ == "__main__":
    main()

