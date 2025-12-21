# Embedding Pipeline for Docusaurus Documentation

This project implements an embedding pipeline that extracts text from deployed Docusaurus URLs, generates embeddings using Cohere, and stores them in Qdrant for RAG-based retrieval.

## Features

- Crawls Docusaurus sites to extract text content
- Cleans and chunks text for optimal embedding
- Generates high-quality embeddings using Cohere
- Stores embeddings in Qdrant vector database
- Single-file implementation for simplicity

## Prerequisites

- Python 3.11 or higher
- UV package manager
- Cohere API key
- Qdrant instance (cloud or self-hosted)

## Setup

1. Clone or create the project directory
2. Install dependencies using UV:
   ```bash
   cd backend
   uv pip install -r requirements.txt
   ```

   Or using pip:
   ```bash
   pip install -r requirements.txt
   ```

3. Create a `.env` file based on `.env.example`:
   ```bash
   cp .env.example .env
   ```

4. Update `.env` with your actual API keys and URLs

## Configuration

The pipeline is configured through environment variables in the `.env` file:

- `COHERE_API_KEY`: Your Cohere API key for generating embeddings
- `QDRANT_URL`: URL of your Qdrant instance
- `QDRANT_API_KEY`: API key for Qdrant (if required)

## Usage

Run the complete pipeline:

```bash
python main.py
```

The pipeline will:
1. Crawl the Docusaurus site (default: https://physical-ai-humanoid-robotics-gilt.vercel.app/)
2. Extract and clean text content from all pages
3. Chunk the content into manageable pieces
4. Generate embeddings using Cohere
5. Create a "teacher_embedding" collection in Qdrant
6. Store the embeddings with metadata in Qdrant

## Architecture

The implementation includes these main functions:

- `get_all_urls`: Crawls the Docusaurus site to find all documentation URLs
- `extract_text_from_url`: Extracts clean text content from a single URL
- `chunk_text`: Splits large text content into smaller chunks
- `embed`: Generates embeddings using the Cohere API
- `create_collection`: Creates a Qdrant collection for storage
- `save_chunk_to_qdrant`: Stores embeddings in Qdrant with metadata
- `main`: Orchestrates the complete pipeline

## Environment Variables

| Variable | Description | Required |
|----------|-------------|----------|
| `COHERE_API_KEY` | Cohere API key for embeddings | Yes |
| `QDRANT_URL` | URL of Qdrant instance | Yes |
| `QDRANT_API_KEY` | Qdrant API key (if needed) | Optional |

## Troubleshooting

- **API Rate Limits**: If you encounter rate limits from Cohere, the code includes delays between API calls
- **Network Issues**: Ensure the target Docusaurus site is accessible
- **Qdrant Connection**: Verify that your Qdrant instance is running and accessible
- **Invalid API Keys**: Double-check your Cohere and Qdrant API keys