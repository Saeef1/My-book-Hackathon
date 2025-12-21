"""
Multi-Provider Agent with Qdrant Retrieval

This module provides functionality to create an AI Agent that can retrieve
information from Qdrant vector database and generate grounded responses to user queries.
The agent supports multiple providers (OpenAI, Google Gemini) and uses custom tools
to connect to Qdrant, perform similarity searches, and incorporate retrieved
information into its responses.
"""

import os
import time
import logging
from typing import Dict, List, Optional, Union
from qdrant_client import QdrantClient
from qdrant_client.http import models
import openai
import google.generativeai as genai
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class OpenAIAgentWithQdrant:
    """
    An OpenAI Agent that integrates with Qdrant for retrieval-augmented generation.
    """

    def __init__(self,
                 openai_api_key: str = None,
                 gemini_api_key: str = None,
                 qdrant_url: str = None,
                 qdrant_api_key: str = None,
                 collection_name: str = None,
                 model: str = "gpt-4-turbo",
                 provider: str = "openai",  # Added provider parameter
                 max_chunks: int = 5,
                 similarity_threshold: float = 0.5,
                 system_prompt: str = None,
                 debug: bool = False,
                 config: Dict = None):
        """
        Initialize the Multi-Provider Agent with Qdrant integration.

        Args:
            openai_api_key: OpenAI API key (defaults to OPENAI_API_KEY env var)
            gemini_api_key: Gemini API key (defaults to GEMINI_API_KEY env var)
            qdrant_url: Qdrant URL (defaults to QDRANT_URL env var)
            qdrant_api_key: Qdrant API key (defaults to QDRANT_API_KEY env var)
            collection_name: Name of the Qdrant collection to search
            model: Model to use for the agent (defaults based on provider)
            provider: AI provider to use ("openai" or "gemini")
            max_chunks: Maximum number of chunks to retrieve from Qdrant
            similarity_threshold: Minimum similarity score for retrieval
            system_prompt: Initial system prompt for the agent
            debug: Enable debug mode for detailed logging
            config: Dictionary with configuration parameters
        """
        # Use config dict if provided, otherwise use individual parameters
        if config:
            openai_api_key = config.get('openai_api_key', os.getenv('OPENAI_API_KEY'))
            gemini_api_key = config.get('gemini_api_key', os.getenv('GEMINI_API_KEY'))
            qdrant_url = config.get('qdrant_url', os.getenv('QDRANT_URL'))
            qdrant_api_key = config.get('qdrant_api_key', os.getenv('QDRANT_API_KEY'))
            collection_name = config.get('collection_name', os.getenv('QDRANT_COLLECTION_NAME', 'teacher_embedding'))
            provider = config.get('provider', 'openai')
            model = config.get('model', 'gpt-4-turbo' if provider == 'openai' else 'gemini-2.0-flash')
            max_chunks = config.get('max_chunks', 5)
            similarity_threshold = config.get('similarity_threshold', 0.5)
            system_prompt = config.get('system_prompt', "You are a helpful AI assistant that answers questions based on retrieved information.")
        else:
            openai_api_key = openai_api_key or os.getenv('OPENAI_API_KEY')
            gemini_api_key = gemini_api_key or os.getenv('GEMINI_API_KEY')
            qdrant_url = qdrant_url or os.getenv('QDRANT_URL')
            qdrant_api_key = qdrant_api_key or os.getenv('QDRANT_API_KEY')
            collection_name = collection_name or os.getenv('QDRANT_COLLECTION_NAME', 'teacher_embedding')
            model = model or ("gpt-4-turbo" if provider == 'openai' else 'gemini-2.0-flash')
            max_chunks = max_chunks or 5
            similarity_threshold = similarity_threshold or 0.5
            system_prompt = system_prompt or "You are a helpful AI assistant that answers questions based on retrieved information."

        # Store configuration
        self.openai_api_key = openai_api_key
        self.gemini_api_key = gemini_api_key
        self.qdrant_url = qdrant_url
        self.qdrant_api_key = qdrant_api_key
        self.collection_name = collection_name
        self.model = model
        self.provider = provider
        self.max_chunks = max_chunks
        self.similarity_threshold = similarity_threshold
        self.system_prompt = system_prompt
        self.debug = debug

        # Initialize appropriate client based on provider
        if provider == "gemini":
            if not gemini_api_key:
                raise ValueError("Gemini API key is required when using Gemini provider")
            genai.configure(api_key=gemini_api_key)
            self.client = genai.GenerativeModel(self.model)
        else:  # openai
            if not openai_api_key:
                raise ValueError("OpenAI API key is required when using OpenAI provider")
            openai.api_key = openai_api_key
            self.client = openai

        # Initialize Qdrant client
        try:
            if qdrant_api_key:
                self.qdrant_client = QdrantClient(url=qdrant_url, api_key=qdrant_api_key)
            else:
                self.qdrant_client = QdrantClient(url=qdrant_url)
            if self.debug:
                logger.info(f"Qdrant client initialized: {qdrant_url}")
        except Exception as e:
            logger.error(f"Failed to initialize Qdrant client: {e}")
            raise

    def retrieve_from_qdrant(self, query_text: str, max_results: int = None, similarity_threshold: float = None) -> List[Dict]:
        """
        Retrieve relevant documents from Qdrant based on the query text.

        Args:
            query_text: The text to search for similar documents
            max_results: Maximum number of results to retrieve (uses default if not provided)
            similarity_threshold: Minimum similarity score (uses default if not provided)

        Returns:
            List of retrieved documents with content and metadata
        """
        max_results = max_results or self.max_chunks
        similarity_threshold = similarity_threshold or self.similarity_threshold

        try:
            # Generate embedding based on the provider
            if self.provider == "gemini":
                # Use Google's embedding API
                result = genai.embed_content(
                    model='text-embedding-004',
                    content=query_text,
                    task_type='retrieval_query'
                )
                query_embedding = result['embedding'][0]
            else:  # openai
                # Use OpenAI's embedding API
                response = openai.embeddings.create(
                    input=query_text,
                    model="text-embedding-ada-002"  # Using a standard embedding model
                )
                query_embedding = response.data[0].embedding

            # Perform search in Qdrant using query_points method (correct API for vector search)
            search_response = self.qdrant_client.query_points(
                collection_name=self.collection_name,
                query=query_embedding,
                limit=max_results,
                score_threshold=similarity_threshold
            )

            # Format results - query_points returns QueryResponse with points attribute
            retrieved_docs = []
            for point in search_response.points:  # Access the points list from QueryResponse
                if point.score >= similarity_threshold:  # Double-check threshold
                    doc = {
                        'id': str(point.id),
                        'content': point.payload.get('content', '') if point.payload else '',
                        'metadata': point.payload or {},
                        'score': point.score,
                        'collection_name': self.collection_name
                    }
                    retrieved_docs.append(doc)

            if self.debug:
                logger.info(f"Retrieved {len(retrieved_docs)} documents from Qdrant")

            return retrieved_docs

        except Exception as e:
            logger.error(f"Failed to retrieve from Qdrant: {e}")
            return []

    def query(self, query_text: str, max_results: int = None, similarity_threshold: float = None) -> Dict:
        """
        Query the agent and get a response based on retrieved information.

        Args:
            query_text: The user's question or query
            max_results: Maximum number of results to retrieve (uses default if not provided)
            similarity_threshold: Minimum similarity score (uses default if not provided)

        Returns:
            Dictionary containing the response and metadata
        """
        start_time = time.time()

        try:
            # Retrieve relevant documents from Qdrant
            retrieved_docs = self.retrieve_from_qdrant(query_text, max_results, similarity_threshold)

            # Build context from retrieved documents
            context_parts = ["Here is the information I found to answer your question:"]
            sources = set()
            for doc in retrieved_docs:
                context_parts.append(f"Document ID: {doc['id']}")
                context_parts.append(f"Content: {doc['content'][:500]}...")  # Limit content length
                if 'source_url' in doc['metadata']:
                    sources.add(doc['metadata']['source_url'])
            context = "\n\n".join(context_parts)

            # Prepare the full prompt with context
            full_prompt = f"{self.system_prompt}\n\nContext:\n{context}\n\nUser Question: {query_text}\n\nPlease provide a detailed answer based on the context provided, and cite sources where appropriate."

            # Generate response based on provider
            if self.provider == "gemini":
                # For Gemini, we use the generative model
                response = self.client.generate_content(
                    contents=full_prompt,
                    generation_config={
                        "max_output_tokens": 1000,
                        "temperature": 0.7
                    }
                )
                response_text = response.text
            else:  # openai
                # Generate response using OpenAI
                response = openai.chat.completions.create(
                    model=self.model,
                    messages=[
                        {"role": "system", "content": self.system_prompt},
                        {"role": "user", "content": full_prompt}
                    ],
                    max_tokens=1000,
                    temperature=0.7
                )
                response_text = response.choices[0].message.content

            execution_time = time.time() - start_time

            return {
                'response_text': response_text,
                'query_id': f"query_{int(time.time())}",
                'execution_time': execution_time,
                'retrieved_documents': retrieved_docs,
                'sources': list(sources),
                'confidence': 0.9 if retrieved_docs else 0.3,  # Higher confidence if we found relevant docs
                'status': 'success'
            }

        except Exception as e:
            logger.error(f"Query failed: {e}")
            execution_time = time.time() - start_time
            return {
                'response_text': f"I'm sorry, I encountered an error processing your request: {str(e)}",
                'query_id': f"query_{int(time.time())}",
                'execution_time': execution_time,
                'retrieved_documents': [],
                'sources': [],
                'confidence': 0.1,
                'status': 'error'
            }

    def test_connection(self) -> bool:
        """
        Test the connection to the AI provider and Qdrant services.

        Returns:
            bool: True if both services are accessible, False otherwise
        """
        try:
            # Test connection to the appropriate AI provider
            if self.provider == "gemini":
                # Test Gemini connection by making a simple call
                response = self.client.generate_content(
                    contents="Hello",
                    generation_config={
                        "max_output_tokens": 5,
                        "temperature": 0.0
                    }
                )
            else:  # openai
                # Test OpenAI connection by making a simple call
                response = openai.chat.completions.create(
                    model=self.model,
                    messages=[{"role": "user", "content": "Hello"}],
                    max_tokens=5,
                    temperature=0.0
                )

            # Test Qdrant connection by listing collections
            collections = self.qdrant_client.get_collections()

            if self.debug:
                logger.info(f"Connected to {self.provider} and Qdrant successfully. Found {len(collections.collections)} collections in Qdrant")

            return True
        except Exception as e:
            logger.error(f"Connection test failed: {e}")
            return False


def test_connection() -> bool:
    """
    Test function to verify agent connections.

    Returns:
        bool: True if connections are successful, False otherwise
    """
    try:
        agent = OpenAIAgentWithQdrant(debug=True)
        return agent.test_connection()
    except Exception as e:
        logger.error(f"Connection test failed: {e}")
        return False


def test_agent_functionality(provider="openai"):
    """
    Test function to verify agent functionality.

    Args:
        provider: The provider to test ("openai" or "gemini")
    """
    print(f"Testing {provider.capitalize()} Agent with Qdrant Retrieval...")

    try:
        # Initialize agent with specified provider
        agent = OpenAIAgentWithQdrant(provider=provider, debug=True)

        # Test connection
        print("1. Testing connections...")
        if agent.test_connection():
            print(f"   SUCCESS: {provider.capitalize()} and Qdrant connections successful")
        else:
            print("   FAILED: Connection failed - please check API keys and service availability")
            return

        # Test basic query (this might not return meaningful results if no data exists)
        print("2. Testing basic query...")
        response = agent.query("What is this system about?", max_results=2)
        print(f"   Response status: {response['status']}")
        print(f"   Retrieved {len(response['retrieved_documents'])} documents")
        if response['sources']:
            print(f"   Sources: {response['sources']}")

        # Test with a longer query
        print("3. Testing with a more specific query...")
        response2 = agent.query("Tell me about best practices for RAG systems", max_results=3)
        print(f"   Response status: {response2['status']}")
        print(f"   Retrieved {len(response2['retrieved_documents'])} documents")
        print(f"   Execution time: {response2['execution_time']:.2f}s")

        print("SUCCESS: All tests completed successfully!")

    except Exception as e:
        print(f"FAILED: Test failed with error: {e}")
        logger.error(f"Test failed: {e}")


def test_gemini_agent():
    """
    Test function specifically for the Gemini agent.
    """
    test_agent_functionality(provider="gemini")


if __name__ == "__main__":
    # Run tests when the module is executed directly
    print("Choose which agent to test:")
    print("1. OpenAI Agent")
    print("2. Gemini Agent")

    choice = input("Enter your choice (1 or 2, default 1): ").strip()

    if choice == "2":
        test_gemini_agent()
    else:
        test_agent_functionality()