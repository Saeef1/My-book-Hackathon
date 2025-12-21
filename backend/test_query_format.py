import os
from qdrant_client import QdrantClient
from qdrant_client.http import models
import cohere

# Initialize clients
qdrant_url = os.getenv("QDRANT_URL")
qdrant_api_key = os.getenv("QDRANT_API_KEY")

if qdrant_api_key:
    qdrant_client = QdrantClient(url=qdrant_url, api_key=qdrant_api_key)
else:
    qdrant_client = QdrantClient(url=qdrant_url)

cohere_client = cohere.Client(os.getenv("COHERE_API_KEY"))

# Generate a test embedding
query_text = "test query"
try:
    query_embedding = cohere_client.embed(
        texts=[query_text],
        model="embed-english-light-v3.0",
        input_type="search_query"
    ).embeddings[0]

    print(f"Query embedding length: {len(query_embedding)}")

    # Perform search
    search_result = qdrant_client.query_points(
        collection_name="teacher_embedding",
        query=query_embedding,
        limit=2,
        with_payload=True
    )

    print(f"Search result type: {type(search_result)}")

    # QueryResponse object has a 'points' attribute
    points = search_result.points
    print(f"Points type: {type(points)}")
    print(f"Points length: {len(points)}")

    if points:
        first_result = points[0]
        print(f"First result type: {type(first_result)}")
        print(f"First result: {first_result}")
        print(f"First result attributes: {[attr for attr in dir(first_result) if not attr.startswith('_')]}")

        # Try to access the payload
        if hasattr(first_result, 'payload'):
            print(f"Payload: {first_result.payload}")
        elif hasattr(first_result, '__getitem__'):
            print(f"Result is subscriptable: {first_result}")
        else:
            print("No payload attribute found")

except Exception as e:
    print(f"Error: {e}")
    import traceback
    traceback.print_exc()