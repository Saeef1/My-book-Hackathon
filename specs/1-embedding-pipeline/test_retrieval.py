"""
Test file for Qdrant retrieval functionality
"""

from backend.retrieve import QdrantRetriever, test_retrieval_functionality

def main():
    print("Starting Qdrant Retrieval Test...")

    # Initialize the retriever
    retriever = QdrantRetriever(debug=True)

    # Test the connection
    print("Testing connection to Qdrant...")
    if retriever.test_connection():
        print("✓ Connected to Qdrant successfully!")

        # List available collections
        collections = retriever.list_collections()
        print(f"Found {len(collections)} collections:")
        for collection in collections:
            print(f"  - {collection['name']} ({collection['count']} items)")

        if collections:
            # Test getting point count for the first collection
            first_collection = collections[0]['name']
            count = retriever.get_point_count(first_collection)
            print(f"Point count in '{first_collection}': {count}")
    else:
        print("✗ Failed to connect to Qdrant")

    print("\nRunning comprehensive tests...")
    test_retrieval_functionality()

if __name__ == "__main__":
    main()