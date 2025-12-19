import cohere
from typing import List, Dict, Optional
from config import settings
import numpy as np


class EmbedderService:
    """
    Service to generate embeddings using Cohere API
    """

    def __init__(self):
        self.co = cohere.Client(settings.cohere_api_key)
        self.model = settings.cohere_embedding_model

    def generate_embeddings(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for a list of texts
        """
        try:
            response = self.co.embed(
                texts=texts,
                model=self.model,
                input_type="search_document"  # Using search_document for content indexing
            )
            return [embedding for embedding in response.embeddings]
        except Exception as e:
            print(f"Error generating embeddings: {e}")
            # Return empty embeddings in case of error - in production, you'd want better error handling
            return [[] for _ in texts]

    def generate_single_embedding(self, text: str) -> List[float]:
        """
        Generate embedding for a single text
        """
        try:
            response = self.co.embed(
                texts=[text],
                model=self.model,
                input_type="search_query"  # Using search_query for user queries
            )
            return response.embeddings[0]
        except Exception as e:
            print(f"Error generating single embedding: {e}")
            return []

    def batch_process_chunks(self, chunks: List[Dict]) -> List[Dict]:
        """
        Process a batch of chunks and add embeddings to them
        """
        if not chunks:
            return []

        # Extract texts for embedding
        texts = [chunk['content'] for chunk in chunks]

        # Generate embeddings
        embeddings = self.generate_embeddings(texts)

        # Add embeddings to chunks
        processed_chunks = []
        for i, chunk in enumerate(chunks):
            chunk_with_embedding = chunk.copy()
            chunk_with_embedding['embedding'] = embeddings[i]
            processed_chunks.append(chunk_with_embedding)

        return processed_chunks


# Example usage
if __name__ == "__main__":
    embedder = EmbedderService()

    # Sample texts
    texts = [
        "This is the first document",
        "This is the second document",
        "This is the third document"
    ]

    # Generate embeddings
    embeddings = embedder.generate_embeddings(texts)
    print(f"Generated {len(embeddings)} embeddings")
    print(f"Embedding dimension: {len(embeddings[0]) if embeddings else 0}")