import os
import cohere
from qdrant_client import QdrantClient
from qdrant_client.http import models
from dotenv import load_dotenv
import logging
from typing import List, Dict, Optional
import requests
from bs4 import BeautifulSoup
from urllib.parse import urljoin

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class RAGBot:
    def __init__(self, collection_name: str = "teacher_embedding"):
        # Initialize Cohere client
        self.cohere_client = cohere.Client(os.getenv("COHERE_API_KEY"))

        # Initialize Qdrant client
        qdrant_url = os.getenv("QDRANT_URL")
        qdrant_api_key = os.getenv("QDRANT_API_KEY")

        if qdrant_api_key:
            self.qdrant_client = QdrantClient(url=qdrant_url, api_key=qdrant_api_key)
        else:
            self.qdrant_client = QdrantClient(url=qdrant_url)

        self.collection_name = collection_name

    def search_similar(self, query: str, top_k: int = 5) -> List[Dict]:
        """
        Search for similar content in the Qdrant collection based on the query
        """
        try:
            # Generate embedding for the query
            query_embedding = self.cohere_client.embed(
                texts=[query],
                model="embed-english-light-v3.0",
                input_type="search_query"
            ).embeddings[0]

            # Search in Qdrant using query_points method (correct API for vector search)
            search_result = self.qdrant_client.query_points(
                collection_name=self.collection_name,
                query=query_embedding,
                limit=top_k,
                with_payload=True
            )

            # Extract relevant information - query_points returns QueryResponse with points attribute
            results = []
            for hit in search_result.points:  # Access the points list from QueryResponse
                results.append({
                    'content': hit.payload.get('content', '') if hit.payload else '',
                    'source_url': hit.payload.get('source_url', ''),
                    'source_title': hit.payload.get('source_title', ''),
                    'score': hit.score
                })

            return results

        except Exception as e:
            logger.error(f"Error during similarity search: {str(e)}")
            return []

    def generate_response(self, query: str, context_chunks: List[Dict], provider: str = "gemini") -> str:
        """
        Generate a response using the query and retrieved context chunks
        Supports only Gemini provider
        """
        try:
            # Prepare context from retrieved chunks
            context_text = "\n\n".join([
                f"Source: {chunk['source_url']}\nContent: {chunk['content']}"
                for chunk in context_chunks
            ])

            # Create a prompt for the language model
            prompt = f"""
            Based on the following documentation context, please answer the user's question.
            If the context doesn't contain relevant information, say so clearly.

            Context:
            {context_text}

            Question: {query}

            Answer:
            """

            if provider == "gemini":
                # Import Google Generative AI here to avoid issues if not installed
                import google.generativeai as genai
                from dotenv import load_dotenv
                import os
                load_dotenv()

                genai.configure(api_key=os.getenv("GEMINI_API_KEY"))
                model = genai.GenerativeModel('gemini-2.5-flash')  # Using gemini-2.5-flash model

                response = model.generate_content(
                    prompt,
                    generation_config=genai.types.GenerationConfig(
                        max_output_tokens=500,
                        temperature=0.3
                    )
                )
                return response.text.strip()

            else:
                # Default to Gemini if provider is not recognized
                import google.generativeai as genai
                from dotenv import load_dotenv
                import os
                load_dotenv()

                genai.configure(api_key=os.getenv("GEMINI_API_KEY"))
                model = genai.GenerativeModel('gemini-2.5-flash')

                response = model.generate_content(
                    prompt,
                    generation_config=genai.types.GenerationConfig(
                        max_output_tokens=500,
                        temperature=0.3
                    )
                )
                return response.text.strip()

        except Exception as e:
            logger.error(f"Error generating response with {provider}: {str(e)}")
            return "Sorry, I encountered an error while generating a response. Please try again."

    def answer_query(self, query: str, top_k: int = 5, provider: str = "gemini") -> Dict:
        """
        Main method to answer a user query using RAG
        """
        logger.info(f"Processing query: {query} with provider: {provider}")

        # Step 1: Search for relevant chunks
        context_chunks = self.search_similar(query, top_k)
        logger.info(f"Found {len(context_chunks)} relevant chunks")

        if not context_chunks:
            return {
                'answer': "I couldn't find any relevant information in the documentation to answer your question.",
                'sources': [],
                'query': query,
                'provider': provider
            }

        # Step 2: Generate response based on context using Gemini
        answer = self.generate_response(query, context_chunks, provider="gemini")

        # Extract sources
        sources = list(set([chunk['source_url'] for chunk in context_chunks if chunk['source_url']]))

        return {
            'answer': answer,
            'sources': sources,
            'query': query,
            'context_chunks': context_chunks
        }

    def check_collection_exists(self) -> bool:
        """
        Check if the collection exists in Qdrant
        """
        try:
            self.qdrant_client.get_collection(self.collection_name)
            return True
        except:
            return False

def main():
    """
    Example usage of the RAG bot
    """
    logger.info("Initializing RAG Bot...")

    # Initialize the RAG bot
    rag_bot = RAGBot(collection_name="teacher_embedding")

    # Check if collection exists
    if not rag_bot.check_collection_exists():
        logger.warning(f"Collection 'teacher_embedding' does not exist. Please run the embedding pipeline first.")
        return

    logger.info("RAG Bot initialized successfully!")

    # Get provider from command line or environment, default to "gemini"
    import sys
    provider = "gemini"  # default
    if len(sys.argv) > 1:
        provider = sys.argv[1].lower()
        if provider != "gemini":
            logger.warning(f"Invalid provider '{provider}', defaulting to 'gemini'")
            provider = "gemini"

    print(f"Using provider: {provider}")

    # Example queries
    queries = [
        "What is the main topic of this documentation?",
        "Tell me about humanoid robotics",
        "How do I get started with the project?",
        "What are the key features?"
    ]

    for query in queries:
        print(f"\nQuery: {query}")
        print("-" * 50)

        result = rag_bot.answer_query(query, provider=provider)

        print(f"Answer: {result['answer']}")

        if result['sources']:
            print(f"Sources: {', '.join(result['sources'][:3])}")  # Show first 3 sources

    # Interactive mode
    print("\n" + "="*60)
    print(f"Interactive mode (using {provider} provider, type 'quit' to exit):")
    while True:
        user_query = input("\nEnter your question: ").strip()
        if user_query.lower() in ['quit', 'exit', 'q']:
            break

        if user_query:
            result = rag_bot.answer_query(user_query, provider=provider)
            print(f"\nAnswer: {result['answer']}")

            if result['sources']:
                print(f"Sources: {', '.join(result['sources'][:3])}")

if __name__ == "__main__":
    main()