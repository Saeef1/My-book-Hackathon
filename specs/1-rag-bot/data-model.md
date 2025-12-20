# Data Model: RAG Bot for Python Data Science Book

## Entity: Document
- **Description**: Represents a web page from the Python Data Science Book
- **Fields**:
  - `id` (string): Unique identifier for the document
  - `url` (string): Source URL of the document
  - `title` (string): Title of the web page
  - `content` (string): Extracted text content from the page
  - `created_at` (datetime): Timestamp when document was indexed
  - `updated_at` (datetime): Timestamp when document was last updated
- **Relationships**:
  - One-to-many with `TextChunk` (one document contains many chunks)
- **Validation**: URL must be valid, content must not be empty

## Entity: TextChunk
- **Description**: A chunk of text from a document, stored as vector embedding
- **Fields**:
  - `id` (string): Unique identifier for the chunk
  - `document_id` (string): Reference to parent document
  - `content` (string): Text content of the chunk
  - `vector` (array[float]): Embedding vector representation
  - `start_pos` (integer): Start position in original document
  - `end_pos` (integer): End position in original document
- **Relationships**:
  - Many-to-one with `Document` (many chunks belong to one document)
- **Validation**: Content must not exceed maximum chunk size, vector must match expected dimensions

## Entity: Query
- **Description**: Represents a user query to the RAG system
- **Fields**:
  - `id` (string): Unique identifier for the query
  - `text` (string): Original query text from user
  - `user_id` (string, optional): Identifier for the user making the query
  - `timestamp` (datetime): When the query was made
  - `provider` (string): AI provider used (openai, gemini, cohere)
- **Relationships**:
  - One-to-many with `QueryResult` (one query produces many results)
- **Validation**: Query text must not be empty, provider must be valid

## Entity: QueryResult
- **Description**: Result of processing a user query
- **Fields**:
  - `id` (string): Unique identifier for the result
  - `query_id` (string): Reference to the original query
  - `response_text` (string): Generated response to the user
  - `sources` (array[string]): List of source URLs used
  - `confidence` (float): Confidence score for the response
  - `execution_time` (float): Time taken to process the query (in seconds)
  - `retrieved_chunks` (array[object]): Chunks used to generate response
  - `status` (string): Status of the query (success, error)
- **Relationships**:
  - Many-to-one with `Query` (many results for one query)
- **Validation**: Response must be provided when status is success

## Entity: EmbeddingModel
- **Description**: Configuration for embedding models used in the system
- **Fields**:
  - `name` (string): Name of the embedding model
  - `provider` (string): Provider of the model (cohere, openai, etc.)
  - `vector_size` (integer): Dimension of the embedding vectors
  - `created_at` (datetime): When this model configuration was created
- **Relationships**:
  - One-to-many with `TextChunk` (one model used for many chunks)
- **Validation**: Vector size must be positive, provider must be supported

## Entity: Session
- **Description**: Represents a user session with the RAG bot
- **Fields**:
  - `id` (string): Unique session identifier
  - `user_id` (string, optional): User identifier
  - `created_at` (datetime): When session started
  - `last_activity` (datetime): When session was last used
  - `query_history` (array[object]): History of queries in this session
- **Relationships**:
  - One-to-many with `Query` (one session contains many queries)
- **Validation**: Session must expire after inactivity period

## State Transitions

### Query Processing States
1. `received` → `processing`: Query received and being processed
2. `processing` → `completed`: Query successfully processed
3. `processing` → `failed`: Query processing failed

### Document Indexing States
1. `pending` → `crawling`: Document URL queued for crawling
2. `crawling` → `extracted`: Content successfully extracted
3. `extracted` → `chunked`: Content chunked for embedding
4. `chunked` → `embedded`: Embeddings generated and stored
5. `embedded` → `indexed`: Document fully indexed in vector store
6. `embedded` → `failed`: Indexing process failed