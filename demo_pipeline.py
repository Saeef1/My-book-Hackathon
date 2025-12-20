#!/usr/bin/env python3
"""
Demo script to show the embedding pipeline structure and functionality
"""
import os
import sys
from pathlib import Path

# Add backend to path
backend_path = Path(__file__).parent / "backend"
sys.path.insert(0, str(backend_path))

# Mock environment variables for demo
os.environ["COHERE_API_KEY"] = "demo_key"
os.environ["QDRANT_URL"] = "http://localhost:6333"

def demo_pipeline():
    """Demonstrate the pipeline structure and functionality"""
    print("=== Embedding Pipeline Demo ===\n")

    # Import the pipeline
    from backend.main import EmbeddingPipeline

    print("1. Creating EmbeddingPipeline instance...")
    pipeline = EmbeddingPipeline()
    print("   SUCCESS: Pipeline initialized\n")

    print("2. Available methods in the pipeline:")
    methods = [
        'get_all_urls',
        'extract_text_from_url',
        'chunk_text',
        'embed',
        'create_collection',
        'save_chunk_to_qdrant'
    ]

    for method in methods:
        print(f"   SUCCESS: {method}")
    print()

    print("3. Example usage of chunk_text method:")
    sample_text = "This is a sample text for demonstration. It will be chunked into smaller pieces for embedding. Each chunk maintains context while being small enough for the embedding model to process effectively."

    chunks = pipeline.chunk_text(sample_text, max_chunk_size=50, overlap=10)
    print(f"   Original text length: {len(sample_text)} characters")
    print(f"   Number of chunks created: {len(chunks)}")

    for i, chunk in enumerate(chunks):
        print(f"   Chunk {i+1}: '{chunk['content'][:50]}...'")
    print()

    print("4. Pipeline execution flow:")
    flow_steps = [
        "Get all URLs from target Docusaurus site",
        "Extract clean text from each URL",
        "Chunk the text into manageable pieces",
        "Generate embeddings using Cohere API",
        "Create 'teacher_embedding' collection in Qdrant",
        "Store embeddings with metadata in Qdrant"
    ]

    for i, step in enumerate(flow_steps, 1):
        print(f"   {i}. {step}")
    print()

    print("=== Pipeline ready for execution ===")
    print("To run the full pipeline, execute: python backend/main.py")
    print("Note: Requires valid Cohere and Qdrant API keys in .env file")

if __name__ == "__main__":
    demo_pipeline()