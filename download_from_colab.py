#!/usr/bin/env python3
"""
Helper script to download NAVA model from Colab
This provides instructions and can help verify the download process
"""

import os
import sys
from pathlib import Path

def print_instructions():
    print("=" * 60)
    print("📥 Download NAVA Model from Colab")
    print("=" * 60)
    print()
    print("The model is located at:")
    print("  /content/nava-llama-3.1-8b-instruct-merged/")
    print()
    print("📋 Step-by-Step Instructions:")
    print()
    print("1. In Colab, click the folder icon (📁) in the left sidebar")
    print("2. Navigate to /content/")
    print("3. Find 'nava-llama-3.1-8b-instruct-merged' folder")
    print("4. Right-click → Download")
    print("   (Or zip first: !zip -r nava-model.zip /content/nava-llama-3.1-8b-instruct-merged)")
    print()
    print("5. Extract the downloaded folder to:")
    print(f"   {Path.home()}/Desktop/NAVA Studio IDE/models/")
    print()
    print("6. Run verification:")
    print("   ./verify_model.sh")
    print()
    print("7. Run setup:")
    print("   ./setup_model_from_colab.sh")
    print()
    print("=" * 60)

def check_download_location():
    """Check if model is already downloaded"""
    project_root = Path.home() / "Desktop" / "NAVA Studio IDE"
    models_dir = project_root / "models"
    
    possible_paths = [
        models_dir / "nava-llama-3.1-8b-instruct-merged",
        models_dir / "nava-instruct-7b-merged",
    ]
    
    for path in possible_paths:
        if path.exists() and (path / "config.json").exists():
            print(f"✅ Found model at: {path}")
            return str(path)
    
    print("❌ Model not found in expected locations")
    return None

def main():
    print_instructions()
    print()
    
    model_path = check_download_location()
    if model_path:
        print()
        print("💡 Model already exists! You can:")
        print(f"   1. Run: ./verify_model.sh {model_path}")
        print(f"   2. Run: ./setup_model_from_colab.sh")
    else:
        print()
        print("💡 Next steps:")
        print("   1. Download model from Colab (see instructions above)")
        print("   2. Extract to models/ directory")
        print("   3. Run: ./verify_model.sh")
        print("   4. Run: ./setup_model_from_colab.sh")

if __name__ == "__main__":
    main()
