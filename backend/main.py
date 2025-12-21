import os
import requests
from bs4 import BeautifulSoup
from urllib.parse import urljoin, urlparse
import cohere
from qdrant_client import QdrantClient
from qdrant_client.http import models
from dotenv import load_dotenv
import time
import logging
from typing import List, Dict, Optional

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class EmbeddingPipeline:
    def __init__(self):
        # Initialize Cohere client
        self.cohere_client = cohere.Client(os.getenv("COHERE_API_KEY"))

        # Initialize Qdrant client
        qdrant_url = os.getenv("QDRANT_URL")
        qdrant_api_key = os.getenv("QDRANT_API_KEY")

        if qdrant_api_key:
            self.qdrant_client = QdrantClient(url=qdrant_url, api_key=qdrant_api_key)
        else:
            self.qdrant_client = QdrantClient(url=qdrant_url)

    def get_all_urls(self, base_url: str, max_depth: int = 3) -> List[str]:
        """
        Crawl a Docusaurus site and return all valid URLs to process
        """
        logger.info(f"Starting to crawl: {base_url}")

        visited_urls = set()
        urls_to_visit = [(base_url, 0)]  # (url, depth)
        valid_urls = []

        while urls_to_visit:
            current_url, depth = urls_to_visit.pop(0)

            if current_url in visited_urls or depth > max_depth:
                continue

            visited_urls.add(current_url)

            try:
                response = requests.get(current_url, timeout=10)
                if response.status_code == 200:
                    # Check if it's an HTML page
                    content_type = response.headers.get('content-type', '')
                    if 'text/html' in content_type:
                        valid_urls.append(current_url)

                        # If we're not at max depth, find more URLs
                        if depth < max_depth:
                            soup = BeautifulSoup(response.text, 'html.parser')
                            for link in soup.find_all('a', href=True):
                                href = link['href']
                                absolute_url = urljoin(current_url, href)

                                # Only add URLs from the same domain
                                if urlparse(absolute_url).netloc == urlparse(base_url).netloc:
                                    if absolute_url not in visited_urls:
                                        urls_to_visit.append((absolute_url, depth + 1))

                time.sleep(0.1)  # Be respectful to the server

            except Exception as e:
                logger.warning(f"Error crawling {current_url}: {str(e)}")
                continue

        logger.info(f"Found {len(valid_urls)} URLs to process")
        return valid_urls

    def extract_text_from_url(self, url: str) -> Optional[Dict]:
        """
        Extract clean text content from a single URL
        """
        try:
            response = requests.get(url, timeout=10)
            if response.status_code == 200:
                soup = BeautifulSoup(response.text, 'html.parser')

                # Remove script and style elements
                for script in soup(["script", "style"]):
                    script.decompose()

                # Try to find main content areas typical in Docusaurus sites
                content_selectors = [
                    'main article',  # Docusaurus main content
                    'main',          # Alternative main
                    'article',       # Article content
                    '.main-wrapper', # Docusaurus specific
                    '.container',    # General container
                    'body'           # Fallback
                ]

                content_element = None
                for selector in content_selectors:
                    content_element = soup.select_one(selector)
                    if content_element:
                        break

                if content_element:
                    # Extract text and clean it up
                    text = content_element.get_text(separator=' ', strip=True)
                    # Clean up extra whitespace
                    text = ' '.join(text.split())
                else:
                    # Fallback to body if no specific content area found
                    text = soup.get_text(separator=' ', strip=True)
                    text = ' '.join(text.split())

                # Extract title
                title_tag = soup.find('title')
                title = title_tag.get_text().strip() if title_tag else url

                return {
                    'title': title,
                    'text': text,
                    'url': url
                }
            else:
                logger.warning(f"Failed to fetch {url}: Status {response.status_code}")
                return None
        except Exception as e:
            logger.error(f"Error extracting text from {url}: {str(e)}")
            return None

    def chunk_text(self, text: str, max_chunk_size: int = 1000, overlap: int = 100) -> List[Dict]:
        """
        Split large text content into smaller chunks for embedding
        """
        if not text:
            return []

        chunks = []
        start = 0
        chunk_id = 0

        while start < len(text):
            # Determine end position
            end = start + max_chunk_size

            # If we're near the end, just take the remainder
            if end > len(text):
                end = len(text)
            else:
                # Try to break at sentence boundary if possible
                possible_end = end
                while possible_end < len(text) and text[possible_end] not in '.!?':
                    possible_end += 1
                if possible_end < len(text):
                    end = possible_end + 1  # Include the sentence ending

            # Extract the chunk
            chunk_text = text[start:end].strip()

            if chunk_text:  # Only add non-empty chunks
                chunks.append({
                    'id': chunk_id,  # Use integer ID instead of string
                    'content': chunk_text,
                    'metadata': {
                        'start_pos': start,
                        'end_pos': end
                    }
                })
                chunk_id += 1

            # Move start position with overlap
            start = end - overlap if end < len(text) else end

        logger.info(f"Text chunked into {len(chunks)} pieces")
        return chunks

    def embed(self, texts: List[str], model: str = "embed-english-light-v3.0") -> List[Dict]:
        """
        Generate embeddings for text chunks using Cohere API
        """
        if not texts:
            return []

        embeddings = []

        # Process in batches to respect API limits
        batch_size = 96  # Cohere's max batch size

        for i in range(0, len(texts), batch_size):
            batch = texts[i:i + batch_size]

            try:
                response = self.cohere_client.embed(
                    texts=batch,
                    model=model,
                    input_type="search_document"
                )

                for j, embedding in enumerate(response.embeddings):
                    embeddings.append({
                        'text_id': f"text_{i + j}",
                        'vector': embedding,
                        'model': model
                    })

                logger.info(f"Embedded batch {i//batch_size + 1}/{(len(texts)-1)//batch_size + 1}")

                # Be respectful to the API
                time.sleep(0.1)

            except Exception as e:
                logger.error(f"Error generating embeddings for batch: {str(e)}")
                # Return partial results if there was an error
                break

        return embeddings

    def create_collection(self, collection_name: str, vector_size: int = 384, distance: str = "Cosine") -> bool:
        """
        Create a Qdrant collection for storing embeddings
        """
        try:
            # Check if collection already exists
            try:
                self.qdrant_client.get_collection(collection_name)
                logger.info(f"Collection '{collection_name}' already exists")
                return True
            except:
                # Collection doesn't exist, so create it
                pass

            # Create the collection
            self.qdrant_client.create_collection(
                collection_name=collection_name,
                vectors_config=models.VectorParams(
                    size=vector_size,
                    distance=models.Distance[distance.upper()]
                )
            )

            logger.info(f"Created collection '{collection_name}' successfully")
            return True

        except Exception as e:
            logger.error(f"Error creating collection '{collection_name}': {str(e)}")
            return False

    def save_chunk_to_qdrant(self, collection_name: str, chunk_id: str, embedding: List[float], metadata: Dict) -> bool:
        """
        Store a text chunk and its embedding in Qdrant
        """
        try:
            # The metadata passed here should include the content field from the chunk
            # Make sure content is properly included in the payload
            self.qdrant_client.upsert(
                collection_name=collection_name,
                points=[
                    models.PointStruct(
                        id=chunk_id,
                        vector=embedding,
                        payload=metadata
                    )
                ]
            )
            return True
        except Exception as e:
            logger.error(f"Error saving chunk {chunk_id} to Qdrant: {str(e)}")
            return False

def main():
    """
    Execute the complete embedding pipeline
    """
    logger.info("Starting embedding pipeline...")

    # Initialize the pipeline
    pipeline = EmbeddingPipeline()

    # Configuration
    site_url = "https://physical-ai-humanoid-robotics-gilt.vercel.app/"
    collection_name = "teacher_embedding"

    logger.info(f"Processing site: {site_url}")

    # Step 1: Get all URLs from the site
    urls = pipeline.get_all_urls(site_url, max_depth=2)
    logger.info(f"Found {len(urls)} URLs to process")

    # Step 2: Extract text from each URL
    documents = []
    for i, url in enumerate(urls):
        logger.info(f"Processing URL {i+1}/{len(urls)}: {url}")
        doc = pipeline.extract_text_from_url(url)
        if doc and doc['text']:
            documents.append(doc)
        time.sleep(0.1)  # Be respectful to the server

    logger.info(f"Successfully processed {len(documents)} documents")

    # Step 3: Chunk the extracted text
    all_chunks = []
    for doc in documents:
        chunks = pipeline.chunk_text(doc['text'], max_chunk_size=1000, overlap=100)
        for chunk in chunks:
            # Add document metadata to chunk
            chunk['metadata'].update({
                'source_url': doc['url'],
                'source_title': doc['title'],
                'document_id': doc['url']
            })
        all_chunks.extend(chunks)

    logger.info(f"Created {len(all_chunks)} text chunks")

    if not all_chunks:
        logger.warning("No content to process - exiting")
        return

    # Step 4: Prepare texts for embedding
    texts_to_embed = [chunk['content'] for chunk in all_chunks]

    # Step 5: Generate embeddings
    logger.info("Generating embeddings...")
    embeddings = pipeline.embed(texts_to_embed)

    if len(embeddings) != len(texts_to_embed):
        logger.error(f"Mismatch: {len(embeddings)} embeddings for {len(texts_to_embed)} texts")
        return

    logger.info(f"Generated {len(embeddings)} embeddings")

    # Step 6: Create Qdrant collection
    vector_size = len(embeddings[0]['vector']) if embeddings else 384
    success = pipeline.create_collection(collection_name, vector_size=vector_size)
    if not success:
        logger.error("Failed to create Qdrant collection")
        return

    # Step 7: Save all chunks to Qdrant
    logger.info("Saving chunks to Qdrant...")
    successful_saves = 0
    for i, (chunk, embedding_info) in enumerate(zip(all_chunks, embeddings)):
        # Combine content with metadata for storage in Qdrant
        full_metadata = chunk['metadata'].copy()
        full_metadata['content'] = chunk['content']  # Add content to metadata

        success = pipeline.save_chunk_to_qdrant(
            collection_name=collection_name,
            chunk_id=chunk['id'],
            embedding=embedding_info['vector'],
            metadata=full_metadata
        )

        if success:
            successful_saves += 1

        if (i + 1) % 10 == 0:  # Log progress every 10 saves
            logger.info(f"Saved {i+1}/{len(all_chunks)} chunks to Qdrant")

    logger.info(f"Successfully saved {successful_saves}/{len(all_chunks)} chunks to Qdrant collection '{collection_name}'")

    logger.info("Embedding pipeline completed successfully!")

if __name__ == "__main__":
    main()