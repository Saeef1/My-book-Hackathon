import unittest
from unittest.mock import Mock, patch
import os
import sys
from pathlib import Path

# Add the parent directory to the path so we can import main
sys.path.insert(0, str(Path(__file__).parent.parent))

from main import EmbeddingPipeline


class TestEmbeddingPipeline(unittest.TestCase):
    def setUp(self):
        # Set up environment variables for testing
        os.environ["COHERE_API_KEY"] = "test_key"
        os.environ["QDRANT_URL"] = "http://localhost:6333"

        self.pipeline = EmbeddingPipeline()

    @patch('requests.get')
    def test_get_all_urls(self, mock_get):
        # Mock a successful response
        mock_response = Mock()
        mock_response.status_code = 200
        mock_response.headers = {'content-type': 'text/html'}
        mock_response.text = '<html><body><a href="/page2">Link</a></body></html>'

        mock_get.return_value = mock_response

        urls = self.pipeline.get_all_urls("http://example.com", max_depth=1)

        # Should return at least the base URL
        self.assertGreaterEqual(len(urls), 1)

    def test_chunk_text(self):
        text = "This is a sample text. It has multiple sentences. We want to chunk it properly."

        chunks = self.pipeline.chunk_text(text, max_chunk_size=30, overlap=5)

        self.assertGreaterEqual(len(chunks), 1)
        # Check that chunks are not empty
        for chunk in chunks:
            self.assertGreater(len(chunk['content']), 0)

    def test_chunk_text_empty(self):
        chunks = self.pipeline.chunk_text("", max_chunk_size=100, overlap=10)

        self.assertEqual(len(chunks), 0)

    def test_chunk_text_shorter_than_chunk_size(self):
        text = "Short text"

        chunks = self.pipeline.chunk_text(text, max_chunk_size=100, overlap=10)

        self.assertEqual(len(chunks), 1)
        self.assertEqual(chunks[0]['content'], text)


if __name__ == '__main__':
    unittest.main()