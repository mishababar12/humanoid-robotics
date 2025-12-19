import uuid
from typing import List, Dict, Optional
from sklearn.feature_extraction.text import TfidfVectorizer
from sklearn.metrics.pairwise import cosine_similarity
import numpy as np
import pickle
import os
from config import settings


class LocalVectorStore:
    """
    A local in-memory vector store that uses TF-IDF for similarity search.
    This serves as a fallback when Qdrant is not available.
    """

    def __init__(self, collection_name: str = None):
        self.collection_name = collection_name or settings.qdrant_collection_name
        self.documents = {}  # id -> document data
        self.contents = []   # list of content strings
        self.ids = []        # list of document IDs
        self.vectorizer = TfidfVectorizer(stop_words='english', max_features=5000)
        self.tfidf_matrix = None
        self._load_from_disk()

    def _get_storage_path(self):
        """Get the path to store the vector data on disk"""
        storage_dir = os.path.join(os.path.dirname(__file__), '..', 'data')
        os.makedirs(storage_dir, exist_ok=True)
        return os.path.join(storage_dir, f"{self.collection_name}_local_store.pkl")

    def _load_from_disk(self):
        """Load stored vectors from disk if available"""
        storage_path = self._get_storage_path()
        if os.path.exists(storage_path):
            try:
                with open(storage_path, 'rb') as f:
                    data = pickle.load(f)
                    self.documents = data.get('documents', {})
                    self.contents = data.get('contents', [])
                    self.ids = data.get('ids', [])

                if self.contents:
                    # Rebuild the TF-IDF matrix
                    self.tfidf_matrix = self.vectorizer.fit_transform(self.contents)
                print(f"Loaded {len(self.contents)} documents from local store")
            except Exception as e:
                print(f"Error loading from disk: {e}")

    def _save_to_disk(self):
        """Save vectors to disk"""
        storage_path = self._get_storage_path()
        try:
            data = {
                'documents': self.documents,
                'contents': self.contents,
                'ids': self.ids
            }
            with open(storage_path, 'wb') as f:
                pickle.dump(data, f)
            print(f"Saved {len(self.contents)} documents to local store")
        except Exception as e:
            print(f"Error saving to disk: {e}")

    def add_document_chunks(self, chunks: List[Dict]) -> bool:
        """
        Add document chunks to the local vector store
        """
        try:
            for chunk in chunks:
                doc_id = chunk.get('id', str(uuid.uuid4()))

                # Store document data
                self.documents[doc_id] = {
                    "content": chunk['content'],
                    "source": chunk.get('source', ''),
                    "title": chunk.get('title', ''),
                    "url": chunk.get('url', ''),
                    "metadata": chunk.get('metadata', {})
                }

                # Add content to list for vectorization
                self.contents.append(chunk['content'])
                self.ids.append(doc_id)

            # Rebuild the TF-IDF matrix with new content
            if self.contents:
                self.tfidf_matrix = self.vectorizer.fit_transform(self.contents)
                self._save_to_disk()

            return True
        except Exception as e:
            print(f"Error adding document chunks to local store: {e}")
            return False

    def search_similar(self, query: str, limit: int = 5, selected_text: Optional[str] = None) -> List[Dict]:
        """
        Search for similar content using TF-IDF and cosine similarity
        """
        try:
            if not self.tfidf_matrix or not self.contents:
                print("Warning: No documents in local store, returning empty results")
                return []

            # Transform the query using the fitted vectorizer
            query_vector = self.vectorizer.transform([query])

            # Calculate cosine similarities
            similarities = cosine_similarity(query_vector, self.tfidf_matrix).flatten()

            # Get the indices of the most similar documents
            similar_indices = similarities.argsort()[::-1][:limit]

            # Prepare results
            results = []
            for idx in similar_indices:
                if similarities[idx] > 0.0:  # Only include results with some similarity
                    doc_id = self.ids[idx]
                    doc_data = self.documents[doc_id]
                    results.append({
                        "id": doc_id,
                        "content": doc_data["content"],
                        "source": doc_data["source"],
                        "title": doc_data["title"],
                        "url": doc_data["url"],
                        "metadata": doc_data["metadata"],
                        "relevance_score": float(similarities[idx])
                    })

            return results
        except Exception as e:
            print(f"Error searching in local store: {e}")
            return []

    def delete_collection(self):
        """Delete the entire collection"""
        try:
            self.documents = {}
            self.contents = []
            self.ids = []
            self.tfidf_matrix = None

            # Delete the stored file
            storage_path = self._get_storage_path()
            if os.path.exists(storage_path):
                os.remove(storage_path)

            return True
        except Exception as e:
            print(f"Error deleting collection: {e}")
            return False

    def get_collection_info(self):
        """Get information about the collection"""
        return {
            "name": self.collection_name,
            "documents_count": len(self.contents),
            "vector_size": self.vectorizer.max_features if self.vectorizer.vocabulary_ else 0
        }


# Global instance
_local_vector_store_instance = None


def get_local_vector_store():
    global _local_vector_store_instance
    if _local_vector_store_instance is None:
        _local_vector_store_instance = LocalVectorStore()
    return _local_vector_store_instance


# For backward compatibility
local_vector_store = get_local_vector_store()