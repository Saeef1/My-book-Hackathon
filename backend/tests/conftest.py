"""
Test configuration file for pytest
"""
import os
import sys
from pathlib import Path

# Add the backend directory to the path so we can import main
backend_path = Path(__file__).parent.parent
sys.path.insert(0, str(backend_path))

# Set up minimal environment variables for testing
os.environ.setdefault("COHERE_API_KEY", "test_key")
os.environ.setdefault("QDRANT_URL", "http://localhost:6333")