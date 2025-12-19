from qdrant_client import QdrantClient
from qdrant_client.http import models
from typing import List, Dict, Optional
from config import settings
import uuid
from .local_vector_store import LocalVectorStore


class QdrantService:
    def __init__(self):
        # Initialize the connected attribute early
        self.connected = False
        self.client = None
        self.collection_name = settings.qdrant_collection_name

        # Try different connection methods for Qdrant Cloud
        # Method 1: Standard cloud connection
        try:
            self.client = QdrantClient(
                url=settings.qdrant_url,
                api_key=settings.qdrant_api_key,
                prefer_grpc=False  # Use HTTP instead of gRPC for cloud
            )
            print("Connected to Qdrant Cloud using HTTP protocol")
        except Exception as e1:
            print(f"HTTP connection failed: {e1}")

            # Method 2: Try with just the host (removing protocol)
            try:
                import re
                clean_url = re.sub(r'^https?://', '', settings.qdrant_url)
                self.client = QdrantClient(
                    host=clean_url,
                    api_key=settings.qdrant_api_key,
                    prefer_grpc=False
                )
                print("Connected to Qdrant using host-only approach")
            except Exception as e2:
                print(f"Host-only connection failed: {e2}")

                # Method 3: Try with HTTPS and HTTP protocol explicitly
                try:
                    self.client = QdrantClient(
                        url=settings.qdrant_url,
                        api_key=settings.qdrant_api_key,
                        https=True  # Explicitly set HTTPS
                    )
                    print("Connected to Qdrant using explicit HTTPS")
                except Exception as e3:
                    print(f"All connection methods failed: {e3}")
                    print("Qdrant functionality will be unavailable until connection is established")
                    self.client = None

        if self.client:
            try:
                self._create_collection_if_not_exists()
                self.connected = True
                print("Qdrant service initialized successfully!")
            except Exception as e:
                print(f"Error setting up collection: {e}")
                self.connected = False
                self.client = None  # Reset client if collection setup fails
        else:
            self.connected = False

    def _create_collection_if_not_exists(self):
        """Create the collection if it doesn't exist"""
        if not self.connected or not self.client:
            return

        try:
            # Try to get the collection first
            collection_info = self.client.get_collection(self.collection_name)
            print(f"Collection '{self.collection_name}' already exists")
        except Exception as e:
            print(f"Collection '{self.collection_name}' does not exist, creating it...")
            try:
                # Create collection with appropriate vector size for Cohere embeddings
                # For Cohere embed-english-v3.0, the vector size should be 1024
                self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=models.VectorParams(
                        size=1024,  # Cohere embed-english-v3.0 returns 1024-dim vectors
                        distance=models.Distance.COSINE
                    ),
                    # Add timeout and other configurations for cloud
                    timeout=60  # Increase timeout for cloud operations
                )
                print(f"Collection '{self.collection_name}' created successfully")
            except Exception as create_error:
                print(f"Error creating collection: {create_error}")
                raise create_error  # Re-raise to be caught by the init method

    def add_document_chunks(self, chunks: List[Dict]) -> bool:
        """
        Add document chunks to the vector database
        Each chunk should have: id, content, metadata (source, title, url, etc.)
        """
        if not self.connected or not self.client:
            print("Error: Cannot add document chunks - Qdrant is not connected")
            return False

        try:
            # Process in batches to avoid connection issues with large uploads
            batch_size = 50  # Smaller batch size to avoid timeouts
            for i in range(0, len(chunks), batch_size):
                batch = chunks[i:i + batch_size]
                points = []

                for chunk in batch:
                    # Use a proper UUID string as the ID instead of potentially problematic values
                    point_id = chunk.get('id', str(uuid.uuid4()))
                    # Ensure the ID is a proper string UUID
                    if not isinstance(point_id, str) or len(point_id) == 0:
                        point_id = str(uuid.uuid4())

                    point = models.PointStruct(
                        id=point_id,
                        vector=chunk['embedding'],
                        payload={
                            "content": chunk['content'],
                            "source": chunk.get('source', ''),
                            "title": chunk.get('title', ''),
                            "url": chunk.get('url', ''),
                            "metadata": chunk.get('metadata', {})
                        }
                    )
                    points.append(point)

                self.client.upsert(
                    collection_name=self.collection_name,
                    points=points
                )

                print(f"Uploaded batch {i//batch_size + 1}/{(len(chunks)-1)//batch_size + 1}")

            return True
        except Exception as e:
            print(f"Error adding document chunks: {e}")
            return False

    def search_similar(self, query_embedding: List[float], limit: int = 5, selected_text: Optional[str] = None) -> List[Dict]:
        """
        Search for similar content in the vector database
        If selected_text is provided, prioritize results from that context
        """
        if not self.connected or not self.client:
            print("Warning: Cannot search - Qdrant is not connected, returning empty results")
            return []

        try:
            # Prepare search filter if selected text context is provided
            search_filter = None
            if selected_text:
                # This is a simplified approach - in a real implementation, you might want to
                # use more sophisticated filtering based on the selected text context
                search_filter = models.Filter(
                    must=[
                        models.FieldCondition(
                            key="content",
                            match=models.MatchText(text=selected_text[:100])  # Limit for performance
                        )
                    ]
                )

            # Perform the search
            search_results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                limit=limit,
                query_filter=search_filter,
                with_payload=True,
                with_vectors=False
            )

            # Format results
            results = []
            for result in search_results:
                results.append({
                    "id": result.id,
                    "content": result.payload.get("content", ""),
                    "source": result.payload.get("source", ""),
                    "title": result.payload.get("title", ""),
                    "url": result.payload.get("url", ""),
                    "metadata": result.payload.get("metadata", {}),
                    "relevance_score": result.score
                })

            return results
        except Exception as e:
            print(f"Error searching similar content: {e}")
            return []

    def delete_collection(self):
        """Delete the entire collection (useful for re-indexing)"""
        try:
            self.client.delete_collection(self.collection_name)
            return True
        except Exception as e:
            print(f"Error deleting collection: {e}")
            return False

    def get_collection_info(self):
        """Get information about the collection"""
        try:
            collection_info = self.client.get_collection(self.collection_name)
            return {
                "name": collection_info.config.params.vectors.size,
                "vector_size": collection_info.config.params.vectors.size,
                "points_count": collection_info.points_count
            }
        except Exception as e:
            print(f"Error getting collection info: {e}")
            return None


# Global instance - will be initialized when first accessed
_qdrant_service_instance = None


def get_qdrant_service():
    global _qdrant_service_instance
    if _qdrant_service_instance is None:
        _qdrant_service_instance = QdrantService()
    return _qdrant_service_instance


# For backward compatibility
qdrant_service = get_qdrant_service()