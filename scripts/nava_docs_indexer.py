#!/usr/bin/env python3
"""
NAVA Documentation Indexer
Builds a searchable index of NAVA documentation from nava-docs-suite
"""

import json
import os
from pathlib import Path
from typing import List, Dict
import re

class NAVADocsIndexer:
    """Index NAVA documentation for semantic search"""
    
    def __init__(self, docs_path: str):
        self.docs_path = Path(docs_path)
        self.index = []
    
    def index_file(self, file_path: Path) -> List[Dict]:
        """Index a single documentation file"""
        entries = []
        
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()
            
            # Extract sections (markdown headers)
            sections = re.split(r'^#+\s+(.+)$', content, flags=re.MULTILINE)
            
            for i in range(1, len(sections), 2):
                if i + 1 < len(sections):
                    title = sections[i].strip()
                    body = sections[i + 1].strip()
                    
                    # Extract code examples
                    code_blocks = re.findall(r'```(?:nava|navlambda)?\n(.*?)```', body, re.DOTALL)
                    
                    # Extract function definitions
                    functions = re.findall(r'`(\w+)\s*\([^)]*\)`', body)
                    
                    entry = {
                        "title": title,
                        "content": body[:500],  # First 500 chars
                        "file": str(file_path.relative_to(self.docs_path)),
                        "code_examples": code_blocks[:3],  # First 3 examples
                        "functions": functions,
                    }
                    entries.append(entry)
        except Exception as e:
            print(f"Error indexing {file_path}: {e}")
        
        return entries
    
    def index_directory(self) -> List[Dict]:
        """Index all documentation files"""
        all_entries = []
        
        # Find all markdown files
        for md_file in self.docs_path.rglob("*.md"):
            entries = self.index_file(md_file)
            all_entries.extend(entries)
        
        # Find all JSON files (if any)
        for json_file in self.docs_path.rglob("*.json"):
            try:
                with open(json_file, 'r') as f:
                    data = json.load(f)
                    if isinstance(data, dict):
                        entry = {
                            "title": data.get("title", json_file.stem),
                            "content": json.dumps(data, indent=2)[:500],
                            "file": str(json_file.relative_to(self.docs_path)),
                            "code_examples": [],
                            "functions": [],
                        }
                        all_entries.append(entry)
            except:
                pass
        
        self.index = all_entries
        return all_entries
    
    def search(self, query: str, max_results: int = 5) -> List[Dict]:
        """Search the index"""
        query_lower = query.lower()
        results = []
        
        for entry in self.index:
            score = 0
            
            # Title match
            if query_lower in entry["title"].lower():
                score += 10
            
            # Content match
            if query_lower in entry["content"].lower():
                score += 5
            
            # Function match
            for func in entry["functions"]:
                if query_lower in func.lower():
                    score += 8
            
            if score > 0:
                results.append((score, entry))
        
        # Sort by score
        results.sort(key=lambda x: x[0], reverse=True)
        
        return [entry for _, entry in results[:max_results]]
    
    def save_index(self, output_path: str):
        """Save index to JSON file"""
        with open(output_path, 'w') as f:
            json.dump(self.index, f, indent=2)
    
    def load_index(self, index_path: str):
        """Load index from JSON file"""
        with open(index_path, 'r') as f:
            self.index = json.load(f)

def main():
    docs_path = "../../Downloads/nava-docs-suite"
    
    if not os.path.exists(docs_path):
        print(f"⚠️  Documentation path not found: {docs_path}")
        print("   Creating mock index for now...")
        indexer = NAVADocsIndexer(docs_path)
        # Create mock entries
        indexer.index = [
            {
                "title": "euclidean_plane()",
                "content": "Creates a 2D Euclidean plane manifold. Returns a Manifold type.",
                "file": "manifolds.md",
                "code_examples": ["let M = euclidean_plane()"],
                "functions": ["euclidean_plane"],
            },
            {
                "title": "navigation_field()",
                "content": "Creates a navigation field on a manifold with start, goal, obstacles, and cost function.",
                "file": "navigation.md",
                "code_examples": ["let field = navigation_field(manifold: M, start: [0,0], goal: [5,5], obstacles: [], cost: geodesic_cost())"],
                "functions": ["navigation_field"],
            },
        ]
    else:
        print(f"📚 Indexing NAVA documentation from {docs_path}...")
        indexer = NAVADocsIndexer(docs_path)
        entries = indexer.index_directory()
        print(f"✅ Indexed {len(entries)} documentation entries")
    
    # Save index
    output_path = "../NAVA Studio IDE/data/nava_docs_index.json"
    indexer.save_index(output_path)
    print(f"💾 Index saved to {output_path}")
    
    # Test search
    print("\n🔍 Testing search...")
    test_queries = ["navigation_field", "euclidean_plane", "obstacle", "cost function"]
    
    for query in test_queries:
        results = indexer.search(query, max_results=3)
        print(f"\nQuery: '{query}'")
        for i, result in enumerate(results, 1):
            print(f"  {i}. {result['title']} ({result['file']})")

if __name__ == "__main__":
    main()

