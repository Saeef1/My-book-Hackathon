import os
import time
from typing import Dict, List, Optional, Union
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import Distance, VectorParams
import numpy as np
import logging
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class QdrantRetriever:
    """
    A class to handle retrieval operations from Qdrant vector database.
    """

    def __init__(self,
                 host: str = None,
                 port: int = None,
                 grpc_port: int = None,
                 https: bool = None,
                 api_key: str = None,
                 timeout: int = None,
                 collection_name: str = None,
                 debug: bool = False,
                 config: Dict = None):
        """
        Initialize the QdrantRetriever with connection parameters.

        Args:
            host: Qdrant host address (default: "localhost")
            port: Qdrant port (default: 6333)
            grpc_port: Qdrant gRPC port (default: 6334)
            https: Whether to use HTTPS (default: False)
            api_key: API key for Qdrant Cloud (optional)
            timeout: Connection timeout in seconds (default: 30)
            collection_name: Default collection name to use
            debug: Enable debug mode for detailed logging
            config: Dictionary with configuration parameters
        """
        # Use config dict if provided, otherwise use individual parameters
        if config:
            host = config.get('host', os.getenv('QDRANT_HOST', 'localhost'))
            port = config.get('port', int(os.getenv('QDRANT_PORT', 6333)))
            grpc_port = config.get('grpc_port', int(os.getenv('QDRANT_GRPC_PORT', 6334)))
            https = config.get('https', os.getenv('QDRANT_HTTPS', 'False').lower() == 'true')
            api_key = config.get('api_key', os.getenv('QDRANT_API_KEY'))
            timeout = config.get('timeout', int(os.getenv('QDRANT_TIMEOUT', 30)))
            collection_name = config.get('collection_name', os.getenv('QDRANT_COLLECTION_NAME', 'teacher_embedding'))
        else:
            host = host or os.getenv('QDRANT_HOST', 'localhost')
            port = port or int(os.getenv('QDRANT_PORT', 6333))
            grpc_port = grpc_port or int(os.getenv('QDRANT_GRPC_PORT', 6334))
            https = https or (os.getenv('QDRANT_HTTPS', 'False').lower() == 'true')
            api_key = api_key or os.getenv('QDRANT_API_KEY')
            timeout = timeout or int(os.getenv('QDRANT_TIMEOUT', 30))
            collection_name = collection_name or os.getenv('QDRANT_COLLECTION_NAME', 'teacher_embedding')

        # Store configuration
        self.host = host
        self.port = port
        self.grpc_port = grpc_port
        self.https = https
        self.api_key = api_key
        self.timeout = timeout
        self.collection_name = collection_name
        self.debug = debug

        # Initialize Qdrant client
        try:
            # Check if QDRANT_URL is set, if so use URL-based initialization
            qdrant_url = os.getenv('QDRANT_URL')
            if qdrant_url:
                if api_key:
                    self.client = QdrantClient(url=qdrant_url, api_key=api_key, timeout=timeout)
                else:
                    self.client = QdrantClient(url=qdrant_url, timeout=timeout)
                if self.debug:
                    logger.info(f"Qdrant client initialized via URL: {qdrant_url}")
            else:
                # Use host/port based initialization
                self.client = QdrantClient(
                    host=host,
                    port=port,
                    grpc_port=grpc_port,
                    https=https,
                    api_key=api_key,
                    timeout=timeout
                )
                if self.debug:
                    logger.info(f"Qdrant client initialized: {host}:{port}")
        except Exception as e:
            logger.error(f"Failed to initialize Qdrant client: {e}")
            raise

    def test_connection(self) -> bool:
        """
        Test the connection to the Qdrant database.

        Returns:
            bool: True if connection is successful, False otherwise
        """
        try:
            # Try to list collections to verify connection
            collections = self.client.get_collections()
            if self.debug:
                logger.info(f"Connected to Qdrant successfully. Found {len(collections.collections)} collections")
            return True
        except Exception as e:
            logger.error(f"Connection test failed: {e}")
            return False

    def list_collections(self) -> List[Dict]:
        """
        List all available collections in Qdrant.

        Returns:
            List[Dict]: List of collection information
        """
        try:
            collections = self.client.get_collections()
            result = []
            for collection in collections.collections:
                # Get collection info to get the vector count
                collection_info = self.client.get_collection(collection.name)
                result.append({
                    'name': collection.name,
                    'size': collection_info.config.params.vectors.size if hasattr(collection_info.config.params.vectors, 'size') else None,
                    'count': collection_info.points_count,
                    'created_at': getattr(collection_info, 'created_at', None)
                })
            return result
        except Exception as e:
            logger.error(f"Failed to list collections: {e}")
            return []

    def retrieve_by_id(self,
                      document_id: str,
                      collection_name: str = None,
                      with_payload: bool = True,
                      with_vectors: bool = False) -> Optional[Dict]:
        """
        Retrieve a specific document by its ID from Qdrant.

        Args:
            document_id: ID of the document to retrieve
            collection_name: Name of the collection (uses default if not provided)
            with_payload: Whether to include payload in response
            with_vectors: Whether to include vectors in response

        Returns:
            Dict: Retrieved document information or None if not found
        """
        collection_name = collection_name or self.collection_name

        try:
            points = self.client.retrieve(
                collection_name=collection_name,
                ids=[document_id],
                with_payload=with_payload,
                with_vectors=with_vectors
            )

            if points and len(points) > 0:
                point = points[0]
                result = {
                    'id': str(point.id),
                    'content': point.payload.get('content', '') if point.payload else '',
                    'embedding': point.vector if with_vectors else None,
                    'metadata': point.payload or {},
                    'collection_name': collection_name,
                    'status': 'success'
                }
                return result
            else:
                logger.warning(f"Document with ID {document_id} not found in collection {collection_name}")
                return None
        except Exception as e:
            logger.error(f"Failed to retrieve document by ID: {e}")
            return None

    def search(self,
               query_text: str = None,
               query_embedding: List[float] = None,
               collection_name: str = None,
               limit: int = 10,
               filters: Dict = None,
               search_params: Dict = None) -> Dict:
        """
        Perform a similarity search in Qdrant.

        Args:
            query_text: Text to search for similar documents (will be ignored if query_embedding is provided)
            query_embedding: Pre-computed embedding to search for similar vectors
            collection_name: Name of the collection (uses default if not provided)
            limit: Maximum number of results to return
            filters: Optional filters to apply to the search
            search_params: Additional search parameters

        Returns:
            Dict: Search results with documents and metadata
        """
        collection_name = collection_name or self.collection_name
        search_params = search_params or {}

        if not query_embedding and not query_text:
            raise ValueError("Either query_text or query_embedding must be provided")

        start_time = time.time()

        try:
            # If query text is provided but no embedding, we would need an embedding model
            # For this retrieval-only implementation, we assume query_embedding is provided
            if query_embedding is None:
                logger.error("Query embedding is required for search")
                return {
                    'results': [],
                    'query_id': None,
                    'execution_time': time.time() - start_time,
                    'total_count': 0,
                    'query_text': query_text,
                    'status': 'error',
                    'error': 'Query embedding is required for search'
                }

            # Prepare filters
            qdrant_filters = None
            if filters:
                filter_conditions = []

                # Add source_url filter if provided
                if 'source_url' in filters:
                    filter_conditions.append(
                        models.FieldCondition(
                            key="source_url",
                            match=models.MatchValue(value=filters['source_url'])
                        )
                    )

                # Add additional metadata filters
                if 'metadata_filters' in filters and filters['metadata_filters']:
                    for key, value in filters['metadata_filters'].items():
                        filter_conditions.append(
                            models.FieldCondition(
                                key=key,
                                match=models.MatchValue(value=value)
                            )
                        )

                if filter_conditions:
                    qdrant_filters = models.Filter(must=filter_conditions)

            # Perform search using query_points method (correct API for vector search)
            search_response = self.client.query_points(
                collection_name=collection_name,
                query=query_embedding,
                limit=limit,
                query_filter=qdrant_filters,
                with_payload=search_params.get('with_payload', True),
                with_vectors=search_params.get('with_vectors', False)
            )

            # Format results - query_points returns QueryResponse with points attribute
            results = []
            for point in search_response.points:  # Access the points list from QueryResponse
                result = {
                    'id': str(point.id),
                    'content': point.payload.get('content', '') if point.payload else '',
                    'embedding': point.vector if search_params.get('with_vectors', False) else None,
                    'metadata': point.payload or {},
                    'score': point.score,
                    'collection_name': collection_name
                }
                results.append(result)

            execution_time = time.time() - start_time

            return {
                'results': results,
                'query_id': f"search_{int(time.time())}",
                'execution_time': execution_time,
                'total_count': len(results),
                'query_text': query_text,
                'status': 'success'
            }

        except Exception as e:
            logger.error(f"Search failed: {e}")
            execution_time = time.time() - start_time
            return {
                'results': [],
                'query_id': None,
                'execution_time': execution_time,
                'total_count': 0,
                'query_text': query_text,
                'status': 'error',
                'error': str(e)
            }

    def get_point_count(self, collection_name: str = None) -> int:
        """
        Get the number of points (documents) in a collection.

        Args:
            collection_name: Name of the collection (uses default if not provided)

        Returns:
            int: Number of points in the collection
        """
        collection_name = collection_name or self.collection_name

        try:
            collection_info = self.client.get_collection(collection_name)
            return collection_info.points_count
        except Exception as e:
            logger.error(f"Failed to get point count: {e}")
            return 0


def test_connection() -> bool:
    """
    Test function to verify Qdrant connection.

    Returns:
        bool: True if connection is successful, False otherwise
    """
    try:
        retriever = QdrantRetriever(debug=True)
        return retriever.test_connection()
    except Exception as e:
        logger.error(f"Connection test failed: {e}")
        return False


def test_retrieval_functionality():
    """
    Test function to verify retrieval functionality.
    """
    print("Testing Qdrant Retrieval Functionality...")

    try:
        # Initialize retriever
        retriever = QdrantRetriever(debug=True)

        # Test connection
        print("1. Testing connection...")
        if retriever.test_connection():
            print("   ✓ Connection successful")
        else:
            print("   ✗ Connection failed")
            return

        # List collections
        print("2. Listing collections...")
        collections = retriever.list_collections()
        print(f"   Found {len(collections)} collections:")
        for collection in collections:
            print(f"   - {collection['name']}: {collection['count']} items")

        # Test retrieval by ID (using a sample ID - this might not exist)
        print("3. Testing document retrieval by ID...")
        # We'll use a sample ID - in real usage, you'd have actual document IDs
        sample_doc = retriever.retrieve_by_id("sample_id_123")
        if sample_doc:
            print(f"   ✓ Retrieved document: {sample_doc.get('content', '')[:50]}...")
        else:
            print("   - Document with sample ID not found (this is expected if no data exists)")

        # Test point count
        if collections:
            first_collection = collections[0]['name']
            count = retriever.get_point_count(first_collection)
            print(f"4. Point count in '{first_collection}': {count}")

        print("✓ All tests completed successfully!")

    except Exception as e:
        print(f"✗ Test failed with error: {e}")
        logger.error(f"Test failed: {e}")


if __name__ == "__main__":
    # Run tests when the module is executed directly
    test_retrieval_functionality()