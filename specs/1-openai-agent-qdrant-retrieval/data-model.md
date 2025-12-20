# Data Model: OpenAI Agent with Qdrant Retrieval

## Overview
This document defines the data structures and models for the OpenAI Agent with Qdrant retrieval system. The system is designed to retrieve embeddings and associated metadata from a Qdrant vector database and use them to generate grounded responses via OpenAI Agents SDK.

## Core Entities

### 1. AgentConfiguration
Configuration for the OpenAI Agent with Qdrant integration.

**Fields:**
- `openai_api_key` (str): API key for OpenAI services
- `qdrant_url` (str): URL for Qdrant database connection
- `qdrant_api_key` (str): API key for Qdrant Cloud (optional)
- `collection_name` (str): Name of the Qdrant collection to search in
- `model` (str): OpenAI model to use for the agent (default: "gpt-4-turbo")
- `max_chunks` (int): Maximum number of chunks to retrieve (default: 5)
- `similarity_threshold` (float): Minimum similarity score for retrieval (default: 0.5)
- `system_prompt` (str): Initial system prompt for the agent

**Validation Rules:**
- `openai_api_key` must be non-empty string
- `qdrant_url` must be a valid URL format
- `max_chunks` must be between 1 and 20
- `similarity_threshold` must be between 0 and 1

### 2. RetrievedDocument
Represents a document retrieved from Qdrant with its associated metadata and embedding.

**Fields:**
- `id` (str): Unique identifier for the document in Qdrant
- `content` (str): The text content of the retrieved document
- `embedding` (list[float]): The vector embedding associated with the document
- `metadata` (dict): Additional metadata stored with the document in Qdrant
  - `source_url` (str): URL where the original content was extracted from
  - `title` (str): Title of the document
  - `section` (str): Section or page identifier
  - `created_at` (str): Timestamp when the document was stored
  - `updated_at` (str): Timestamp when the document was last updated
- `score` (float): Similarity score when retrieved via similarity search
- `collection_name` (str): Name of the Qdrant collection where the document is stored

**Validation Rules:**
- `id` must be non-empty string
- `content` must be non-empty string
- `embedding` must be a list of floats with consistent dimensions
- `score` must be between 0 and 1 (for similarity scores)

### 3. AgentQuery
Represents a query request to the agent system.

**Fields:**
- `query_text` (str): The user's question or query text
- `user_id` (str): Identifier for the user (optional)
- `session_id` (str): Identifier for the conversation session (optional)
- `retrieval_params` (dict): Parameters for the retrieval process
  - `max_results` (int): Maximum number of results to retrieve
  - `similarity_threshold` (float): Minimum similarity threshold
  - `filters` (dict): Additional filters to apply to the search

**Validation Rules:**
- `query_text` must be non-empty string
- `max_results` must be between 1 and 20

### 4. AgentResponse
Represents the response from the agent system.

**Fields:**
- `response_text` (str): The agent's response to the user's query
- `query_id` (str): Unique identifier for the query
- `execution_time` (float): Time taken to process the query in milliseconds
- `retrieved_documents` (list[RetrievedDocument]): List of documents used to generate the response
- `sources` (list[str]): List of source identifiers for attribution
- `confidence` (float): Agent's confidence in the response (0-1)
- `status` (str): Status of the query execution ("success", "error", "partial")

**Validation Rules:**
- `response_text` must be non-empty string
- `execution_time` must be non-negative
- `confidence` must be between 0 and 1

### 5. QdrantConnectionConfig
Configuration for connecting to Qdrant.

**Fields:**
- `host` (str): Qdrant host address (default: "localhost")
- `port` (int): Qdrant port (default: 6333)
- `grpc_port` (int): Qdrant gRPC port (default: 6334)
- `https` (bool): Whether to use HTTPS (default: False)
- `api_key` (str): API key for Qdrant Cloud (optional)
- `timeout` (int): Connection timeout in seconds (default: 30)
- `collection_name` (str): Default collection name to use

**Validation Rules:**
- `host` must be a valid host address
- `port` must be between 1 and 65535
- `timeout` must be positive

## Relationships

### AgentResponse and RetrievedDocument
- An `AgentResponse` contains multiple `RetrievedDocument` instances in the `retrieved_documents` field
- One-to-many relationship (one response, many supporting documents)

### AgentQuery and AgentConfiguration
- An `AgentQuery` uses an `AgentConfiguration` for processing
- References configuration parameters for the agent execution

### RetrievedDocument and QdrantConnectionConfig
- A `RetrievedDocument` comes from a Qdrant collection specified in `QdrantConnectionConfig`
- Retrieved documents contain collection name for reference

## Data Flow

1. **Query Input**: User provides a question/query to the agent
2. **Configuration**: System loads agent configuration with API keys and parameters
3. **Query Embedding**: System converts the user query to an embedding vector
4. **Similarity Search**: System performs similarity search in Qdrant collection
5. **Retrieval**: Matching documents with content, metadata, and similarity scores are retrieved
6. **Context Building**: Retrieved documents are formatted and added to agent context
7. **Response Generation**: Agent generates a response based on the retrieved information
8. **Response Output**: System returns the agent response with source attribution

## Validation Rules

### AgentConfiguration
- API keys must be provided and valid
- Model name must be a supported OpenAI model
- Collection name must exist in Qdrant

### RetrievedDocument
- Content must not be empty
- Score must be between 0 and 1
- ID must be unique within the collection

### AgentResponse
- Response text must not be empty when status is "success"
- Retrieved documents must be properly formatted
- Confidence score must be between 0 and 1

## Serialization Format

The system will support JSON serialization for all data models to enable easy data exchange and debugging.

### JSON Schema for AgentResponse:
```json
{
  "response_text": "string",
  "query_id": "string",
  "execution_time": 125.5,
  "retrieved_documents": [
    {
      "id": "string",
      "content": "string",
      "metadata": {
        "source_url": "string",
        "title": "string"
      },
      "score": 0.85,
      "collection_name": "string"
    }
  ],
  "sources": ["string"],
  "confidence": 0.95,
  "status": "success"
}
```

## Constraints

1. Embedding dimensions must match the collection schema in Qdrant
2. Retrieved documents must not exceed token limits for the OpenAI model
3. Connection configurations must be validated before use
4. All timestamps should follow ISO 8601 format
5. Response generation must include proper source attribution