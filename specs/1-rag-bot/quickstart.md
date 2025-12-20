# Quickstart: RAG Bot for Python Data Science Book

## Prerequisites

- Python 3.11+
- Node.js 18+ (for frontend development)
- Access to Qdrant vector database (cloud or local)
- API keys for at least one AI provider (OpenAI, Google Gemini, or Cohere)
- Access to the Python Data Science Book website for indexing

## Setup

### 1. Clone and Install Backend Dependencies

```bash
# Navigate to the backend directory
cd backend

# Create and activate virtual environment
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt
```

### 2. Configure Environment Variables

Create a `.env` file in the backend directory with the following variables:

```env
# Qdrant Configuration
QDRANT_URL=your-qdrant-url
QDRANT_API_KEY=your-qdrant-api-key  # if using cloud

# AI Provider Keys (at least one required)
OPENAI_API_KEY=your-openai-api-key
GEMINI_API_KEY=your-gemini-api-key
COHERE_API_KEY=your-cohere-api-key

# Qdrant Collection Name
QDRANT_COLLECTION_NAME=python_data_science_book
```

### 3. Run the Embedding Pipeline

```bash
# Index the Python Data Science Book content
python main.py
```

This will:
- Crawl the Python Data Science Book website
- Extract text content from pages
- Generate embeddings using Cohere
- Store in Qdrant vector database

### 4. Start the Backend API

```bash
# Run the API server
python app.py
```

The API will be available at `http://localhost:8000`

## Frontend Integration

### 1. Install Docusaurus Dependencies

```bash
# Navigate to frontend directory (assuming Docusaurus setup)
cd frontend  # or wherever your Docusaurus project is

# Install dependencies
npm install
```

### 2. Add RAG Bot Component

Create the RAG bot component in your Docusaurus project:

```bash
# Create component directory
mkdir -p src/components/RagBot
```

### 3. Run Development Server

```bash
npm run start
```

## Testing the RAG Bot

### 1. Verify Backend Functionality

Test the API endpoints:

```bash
# Check system status
curl http://localhost:8000/api/status

# Submit a test query
curl -X POST http://localhost:8000/api/query \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is pandas used for in data science?",
    "provider": "openai"
  }'
```

### 2. Test Directly in Python

```python
from rag_bot import RAGBot

# Initialize the bot
rag_bot = RAGBot(collection_name="python_data_science_book")

# Test a query
result = rag_bot.answer_query("What is pandas used for?")
print(result['answer'])
print("Sources:", result['sources'])
```

## Frontend Integration Example

To integrate the RAG bot into a Docusaurus page:

```jsx
// Example React component for Docusaurus
import React, { useState } from 'react';
import RagBotComponent from '../components/RagBot';

function DataSciencePage() {
  return (
    <div>
      <h1>Python Data Science Book</h1>
      {/* Your page content */}
      <div className="rag-bot-container">
        <RagBotComponent />
      </div>
    </div>
  );
}
```

## Configuration Options

### API Configuration

The RAG bot supports multiple configuration options:

- **AI Provider**: Choose between OpenAI, Gemini, or Cohere for response generation
- **Max Results**: Number of context chunks to retrieve (default: 5)
- **Similarity Threshold**: Minimum similarity score for retrieved documents (default: 0.5)

### Environment Variables

| Variable | Description | Required |
|----------|-------------|----------|
| QDRANT_URL | URL for Qdrant vector database | Yes |
| QDRANT_API_KEY | API key for Qdrant (if using cloud) | No* |
| OPENAI_API_KEY | OpenAI API key | No^ |
| GEMINI_API_KEY | Google Gemini API key | No^ |
| COHERE_API_KEY | Cohere API key | No^ |
| QDRANT_COLLECTION_NAME | Name of the Qdrant collection | No |

*Required if using Qdrant Cloud
^At least one AI provider key is required

## Troubleshooting

### Common Issues

1. **API Keys Not Working**: Verify that your API keys are correct and have the necessary permissions.

2. **Qdrant Connection Issues**: Check that your QDRANT_URL is correct and accessible.

3. **No Results Found**: Ensure the embedding pipeline completed successfully and indexed content is available.

4. **Rate Limiting**: If you encounter rate limiting, implement appropriate delays or upgrade your API plan.

### Verification Steps

1. Check that the Qdrant collection exists and contains indexed documents
2. Verify that API keys are properly configured
3. Test the embedding pipeline independently before using the RAG bot
4. Monitor logs for any error messages during query processing

## Next Steps

1. Customize the RAG bot UI to match your Docusaurus theme
2. Add additional configuration options as needed
3. Implement analytics to track query patterns and user satisfaction
4. Set up automated indexing to keep content up-to-date