from typing import List, Dict, Optional
import sys
from pathlib import Path

# Add backend directory to path for imports
backend_dir = Path(__file__).parent.parent
if str(backend_dir) not in sys.path:
    sys.path.insert(0, str(backend_dir))

from database.qdrant import get_qdrant_service
from ingestion.embedder import EmbedderService
from config import settings


class SearchService:
    """
    Service to handle search functionality for the RAG chatbot
    """

    def __init__(self):
        self.embedder = EmbedderService()
        self.qdrant = get_qdrant_service()

    def search_content(self, query: str, selected_text: Optional[str] = None, limit: int = None) -> List[Dict]:
        """
        Search for relevant content based on query and optional selected text context
        """
        if limit is None:
            limit = settings.max_search_results

        print(f"\n=== DEBUG: Searching for query: '{query}' ===")

        # Generate embedding for the query
        query_embedding = self.embedder.generate_single_embedding(query)
        if not query_embedding:
            print("ERROR: Could not generate embedding for query")
            return []

        print(f"Generated embedding of length: {len(query_embedding)}")

        # Perform search in vector database
        search_results = self.qdrant.search_similar(
            query_embedding=query_embedding,
            limit=limit,
            selected_text=selected_text
        )

        print(f"Found {len(search_results)} results from Qdrant")
        print("Top results details:")
        for i, result in enumerate(search_results[:3]):  # Print top 3 results
            rank = i + 1
            score = result.get("relevance_score", 0.0)
            content = result.get("content", "")[:300]  # First 300 chars
            title = result.get("title", "Unknown")
            url = result.get("url", "Unknown")
            source = result.get("source", "Unknown")

            print(f"  Rank {rank}:")
            print(f"    Score: {score:.4f}")
            print(f"    Title: {title}")
            print(f"    URL: {url}")
            print(f"    Source: {source}")
            print(f"    Content preview: {content}...")
            print()

        return search_results

    def search_with_context(self, query: str, context_text: Optional[str] = None) -> List[Dict]:
        """
        Search with additional context from selected text
        """
        # If context_text is provided, combine it with the query for better search
        if context_text:
            enhanced_query = f"{context_text} {query}"
        else:
            enhanced_query = query

        return self.search_content(
            query=enhanced_query,
            selected_text=context_text
        )

    def get_content_by_source(self, source_url: str) -> List[Dict]:
        """
        Retrieve content chunks by source URL (useful for related content)
        """
        # This would be implemented based on how we store and query by source
        # For now, we'll return an empty list as this requires more complex filtering
        # that may not be directly supported by Qdrant without custom payload structure
        return []


# Global instance
search_service = SearchService()