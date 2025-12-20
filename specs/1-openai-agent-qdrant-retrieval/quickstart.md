# Quickstart Guide: OpenAI Agent with Qdrant Retrieval

## Overview
This guide provides instructions for quickly setting up and using the OpenAI Agent with Qdrant retrieval functionality. The agent retrieves information from Qdrant vector database and generates grounded responses using the OpenAI Agents SDK.

## Prerequisites
- Python 3.8 or higher
- OpenAI API key
- Qdrant vector database running (local or remote)
- `openai`, `qdrant-client`, `python-dotenv` Python packages
- Embeddings already stored in Qdrant collection

## Installation

### 1. Install Dependencies
```bash
uv pip install openai qdrant-client python-dotenv
```

### 2. Set Up Environment
Create a `.env` file in your project root:
```env
OPENAI_API_KEY=your_openai_api_key_here
QDRANT_URL=your_qdrant_url_here
QDRANT_API_KEY=your_qdrant_api_key_here  # Optional, for Qdrant Cloud
QDRANT_COLLECTION_NAME=teacher_embedding
```

## Basic Usage

### 1. Import and Initialize
```python
from backend.agent import OpenAIAgentWithQdrant

# Initialize the agent with default configuration
agent = OpenAIAgentWithQdrant()
```

### 2. Query the Agent
```python
# Ask a question that requires information from Qdrant
response = agent.query("What are the best practices for RAG systems?")
print(f"Agent response: {response['response_text']}")
print(f"Sources used: {response['sources']}")
```

### 3. Advanced Query with Parameters
```python
# Perform a query with custom retrieval parameters
response = agent.query(
    query="How do I implement document chunking?",
    max_results=3,
    similarity_threshold=0.8
)

print(f"Response: {response['response_text']}")
print(f"Retrieved {len(response['retrieved_documents'])} documents")
```

### 4. Test Agent Connectivity
```python
# Test if all services are connected properly
if agent.test_connection():
    print("All services connected successfully!")
else:
    print("Connection issues detected")
```

## Configuration Options

### Using Custom Configuration
```python
config = {
    'openai_api_key': 'your_openai_key',
    'qdrant_url': 'https://your-qdrant-url.com',
    'collection_name': 'my_collection',
    'model': 'gpt-4-turbo',
    'max_chunks': 5
}

agent = OpenAIAgentWithQdrant(config=config)
```

### Using Environment Variables
```python
import os
from backend.agent import OpenAIAgentWithQdrant

agent = OpenAIAgentWithQdrant(
    openai_api_key=os.getenv('OPENAI_API_KEY'),
    qdrant_url=os.getenv('QDRANT_URL'),
    collection_name=os.getenv('QDRANT_COLLECTION_NAME', 'teacher_embedding')
)
```

## Testing the System

### Run Basic Tests
```python
from backend.agent import test_agent_functionality

# Run all basic tests
test_agent_functionality()
```

### Test Connection Only
```python
from backend.agent import test_connection

if test_connection():
    print("All systems operational!")
else:
    print("Connection issues detected")
```

## Integration with Existing Pipeline

The agent can work with your existing embedding pipeline:

1. Use `backend/main.py` to store documents in Qdrant
2. Use `backend/retrieve.py` to verify stored content
3. Use `backend/agent.py` to create an AI agent that retrieves from Qdrant

### Example Integration Flow:
```python
# 1. First, ensure documents are stored using the embedding pipeline
# python backend/main.py

# 2. Then use the agent to retrieve and answer questions
from backend.agent import OpenAIAgentWithQdrant

agent = OpenAIAgentWithQdrant()
response = agent.query("Summarize the key points about embedding pipelines")
print(response['response_text'])
```

## Troubleshooting

### Common Issues

1. **API Connection Refused**
   - Verify OpenAI API key is correct
   - Check Qdrant URL and API key
   - Ensure both services are accessible

2. **No Relevant Results**
   - Verify embeddings exist in the Qdrant collection
   - Check if query matches the content in your stored documents
   - Adjust similarity threshold if needed

3. **Rate Limiting**
   - OpenAI API may have rate limits
   - Implement appropriate delays between requests
   - Check your OpenAI usage limits

### Debug Mode
Enable debug mode to get more detailed information:
```python
agent = OpenAIAgentWithQdrant(debug=True)
```

## Next Steps

1. Integrate the agent into your application
2. Implement conversation history management
3. Add more sophisticated retrieval strategies
4. Monitor performance and optimize queries as needed
5. Implement proper error handling for production use