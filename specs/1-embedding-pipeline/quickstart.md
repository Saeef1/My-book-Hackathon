# Quickstart Guide: Embedding Pipeline

## Prerequisites

Before getting started, ensure you have:
- Python 3.11 or higher installed
- UV package manager installed
- Access to Cohere API (API key)
- Access to Qdrant (either hosted or self-hosted instance)

## Installation

1. **Clone or create the backend directory:**
   ```bash
   mkdir backend
   cd backend
   ```

2. **Initialize the project with UV:**
   ```bash
   uv init
   ```

3. **Install required dependencies:**
   ```bash
   uv add requests beautifulsoup4 cohere qdrant-client python-dotenv
   ```

4. **Set up environment variables:**
   Create a `.env` file in the backend directory with the following content:
   ```env
   COHERE_API_KEY=your_cohere_api_key_here
   QDRANT_URL=your_qdrant_url_here
   QDRANT_API_KEY=your_qdrant_api_key_here  # if using cloud service
   ```

## Configuration

1. **Create the main.py file** with the complete pipeline implementation as specified.

2. **Environment Variables:**
   - `COHERE_API_KEY`: Your Cohere API key for generating embeddings
   - `QDRANT_URL`: URL of your Qdrant instance
   - `QDRANT_API_KEY`: API key for Qdrant (if required)

## Running the Pipeline

1. **Execute the main script:**
   ```bash
   cd backend
   python main.py
   ```

2. **The pipeline will:**
   - Crawl the specified Docusaurus site (https://physical-ai-humanoid-robotics-gilt.vercel.app/)
   - Extract and clean text content from all pages
   - Chunk the content into manageable pieces
   - Generate embeddings using Cohere
   - Create a "teacher_embedding" collection in Qdrant
   - Store the embeddings with metadata in Qdrant

## Verification

After running the pipeline, you can verify:
1. The "teacher_embedding" collection exists in Qdrant
2. Embeddings have been stored successfully
3. The console output shows successful processing of documents

## Troubleshooting

- **API Rate Limits**: If you encounter rate limits from Cohere, consider adding delays between API calls
- **Network Issues**: Ensure the target Docusaurus site is accessible
- **Qdrant Connection**: Verify that your Qdrant instance is running and accessible
- **Invalid API Keys**: Double-check your Cohere and Qdrant API keys

## Next Steps

- Customize the URL patterns to crawl different sections of your documentation
- Adjust chunking parameters based on your content needs
- Implement incremental updates to only process new or changed content
- Use the retrieve.py module to access stored embeddings for RAG applications

## Using the Retrieval System

After running the embedding pipeline to store your documentation in Qdrant, you can use the retrieval system to access the stored embeddings:

### 1. Basic Retrieval Setup
```bash
cd backend
python -c "from retrieve import QdrantRetriever; r = QdrantRetriever(); print('Retriever ready!')"
```

### 2. Connect to Qdrant and Test
```python
from backend.retrieve import QdrantRetriever

# Initialize the retriever with default configuration
retriever = QdrantRetriever()

# Test the connection
if retriever.test_connection():
    print("Successfully connected to Qdrant!")
else:
    print("Failed to connect to Qdrant")
```

### 3. Retrieve Specific Documents
```python
# Retrieve a document by its ID
document = retriever.retrieve_by_id("chunk_0", collection_name="teacher_embedding")
if document:
    print(f"Content: {document['content']}")
    print(f"Metadata: {document['metadata']}")
```

### 4. Perform Similarity Search
```python
# Perform a similarity search (you'll need an embedding vector)
# This example shows the structure - you'd need actual embedding values
query_embedding = [0.1, 0.2, 0.3, ...] # Your actual embedding vector

results = retriever.search(
    query_embedding=query_embedding,
    collection_name="teacher_embedding",
    limit=5
)

for doc in results['results']:
    print(f"Score: {doc['score']}")
    print(f"Content: {doc['content'][:100]}...")
```

### 5. List Available Collections
```python
collections = retriever.list_collections()
for collection in collections:
    print(f"Collection: {collection['name']}, Count: {collection['count']}")
```

For more detailed usage examples, refer to the test functions in `backend/retrieve.py`.