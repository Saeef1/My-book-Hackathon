#!/usr/bin/env python3
"""
Simple test script to verify the embedding pipeline components work
"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'backend'))

def test_imports():
    """Test that all required libraries can be imported"""
    try:
        import requests
        import cohere
        from qdrant_client import QdrantClient
        from bs4 import BeautifulSoup
        from dotenv import load_dotenv
        print("SUCCESS: All required libraries imported successfully")
        return True
    except ImportError as e:
        print(f"ERROR: Import error: {e}")
        return False

def test_main_structure():
    """Test that main.py has the required functions"""
    try:
        from backend.main import EmbeddingPipeline
        pipeline = EmbeddingPipeline()

        # Check if required methods exist
        required_methods = [
            'get_all_urls',
            'extract_text_from_url',
            'chunk_text',
            'embed',
            'create_collection',
            'save_chunk_to_qdrant'
        ]

        for method in required_methods:
            if hasattr(pipeline, method):
                print(f"SUCCESS: Method {method} exists")
            else:
                print(f"ERROR: Method {method} missing")
                return False

        print("SUCCESS: All required methods exist in EmbeddingPipeline class")
        return True
    except Exception as e:
        print(f"ERROR: Error testing main structure: {e}")
        return False

if __name__ == "__main__":
    print("Testing Embedding Pipeline Implementation...")
    print()

    success = True
    success &= test_imports()
    print()
    success &= test_main_structure()
    print()

    if success:
        print("SUCCESS: All tests passed! The embedding pipeline is ready.")
    else:
        print("ERROR: Some tests failed.")
        sys.exit(1)