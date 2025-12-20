# Internal API Contract: Embedding Pipeline Functions

## Function: get_all_urls
**Purpose**: Crawl a Docusaurus site and return all valid URLs to process

**Input Parameters**:
- `base_url` (string): The root URL of the Docusaurus site to crawl
- `max_depth` (int, optional): Maximum depth to crawl (default: 3)

**Return Value**:
- `urls` (list[string]): List of all discovered URLs that contain documentation content

**Error Handling**:
- Raises exception if base_url is invalid or inaccessible

## Function: extract_text_from_url
**Purpose**: Extract clean text content from a single URL

**Input Parameters**:
- `url` (string): The URL to extract text from

**Return Value**:
- `content` (dict): Contains:
  - `title` (string): Page title
  - `text` (string): Clean, extracted text content
  - `url` (string): Source URL

**Error Handling**:
- Returns None if URL is inaccessible
- Strips HTML tags, navigation elements, and other non-content elements

## Function: chunk_text
**Purpose**: Split large text content into smaller chunks for embedding

**Input Parameters**:
- `text` (string): The text to be chunked
- `max_chunk_size` (int, optional): Maximum size of each chunk in characters (default: 1000)
- `overlap` (int, optional): Number of characters to overlap between chunks (default: 100)

**Return Value**:
- `chunks` (list[dict]): List of chunks, each containing:
  - `id` (string): Unique chunk identifier
  - `content` (string): The chunked text
  - `metadata` (dict): Additional metadata about the chunk

## Function: embed
**Purpose**: Generate embeddings for text chunks using Cohere API

**Input Parameters**:
- `texts` (list[string]): List of text chunks to embed
- `model` (string, optional): Cohere model to use (default: "embed-english-light-v3.0")

**Return Value**:
- `embeddings` (list[dict]): List of embeddings, each containing:
  - `text_id` (string): Reference to the original text chunk
  - `vector` (list[float]): The embedding vector
  - `model` (string): Model used for embedding

**Error Handling**:
- Handles API rate limiting with appropriate delays
- Returns error information if API call fails

## Function: create_collection
**Purpose**: Create a Qdrant collection for storing embeddings

**Input Parameters**:
- `collection_name` (string): Name of the collection to create (e.g., "teacher_embedding")
- `vector_size` (int): Dimension of the vectors to be stored
- `distance` (string, optional): Distance metric (default: "Cosine")

**Return Value**:
- Boolean indicating success or failure

**Error Handling**:
- Handles collection creation errors
- Checks if collection already exists

## Function: save_chunk_to_qdrant
**Purpose**: Store a text chunk and its embedding in Qdrant

**Input Parameters**:
- `collection_name` (string): Name of the collection to store in
- `chunk_id` (string): Unique identifier for the chunk
- `embedding` (list[float]): The embedding vector
- `metadata` (dict): Metadata to store with the chunk

**Return Value**:
- Boolean indicating success or failure

**Error Handling**:
- Handles storage errors
- Validates embedding dimensions match collection requirements

## Function: main
**Purpose**: Execute the complete embedding pipeline

**Input Parameters**:
- `site_url` (string): URL of the Docusaurus site to process
- `collection_name` (string, optional): Name of the Qdrant collection (default: "teacher_embedding")

**Return Value**:
- None (executes the complete pipeline)

**Execution Flow**:
1. Gets all URLs from the site
2. Extracts text from each URL
3. Chunks the extracted text
4. Generates embeddings for all chunks
5. Creates the Qdrant collection
6. Saves all chunks to Qdrant