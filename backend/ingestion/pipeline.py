from .extractor import ContentExtractor
from .chunker import ContentChunker
from .embedder import EmbedderService
from database.qdrant import qdrant_service
from typing import List, Dict
import logging
from config import settings


class IngestionPipeline:
    """
    Complete pipeline for ingesting, processing, and indexing book content
    """

    def __init__(self, docs_path: str = "my-website/docs"):
        self.extractor = ContentExtractor(docs_path)
        self.chunker = ContentChunker(
            chunk_size=settings.chunk_size,
            chunk_overlap=settings.chunk_overlap
        )
        self.embedder = EmbedderService()

    def run_full_ingestion(self) -> bool:
        """
        Run the complete ingestion pipeline:
        1. Extract content from Docusaurus docs
        2. Chunk the content
        3. Generate embeddings
        4. Store in vector database
        """
        try:
            print("Starting content extraction...")
            content = self.extractor.extract_all_content()
            print(f"Extracted {len(content)} content sections")

            if not content:
                print("No content found to process")
                return False

            print("Chunking content...")
            chunks = self.chunker.chunk_content(content)
            print(f"Created {len(chunks)} content chunks")

            print("Generating embeddings...")
            chunks_with_embeddings = self.embedder.batch_process_chunks(chunks)
            print("Embeddings generated successfully")

            print("Storing in vector database...")
            success = self._store_in_vector_db(chunks_with_embeddings)
            if success:
                print("Content successfully stored in vector database")
            else:
                print("Failed to store content in vector database")
                return False

            return True
        except Exception as e:
            logging.error(f"Error in ingestion pipeline: {e}")
            return False

    def _store_in_vector_db(self, chunks: List[Dict]) -> bool:
        """
        Store processed chunks in the vector database
        """
        try:
            # Add all chunks to Qdrant
            success = qdrant_service.add_document_chunks(chunks)
            return success
        except Exception as e:
            logging.error(f"Error storing chunks in vector database: {e}")
            return False

    def update_content(self, new_content: List[Dict]) -> bool:
        """
        Update vector database with new content
        """
        try:
            # Chunk the new content
            chunks = self.chunker.chunk_content(new_content)
            print(f"Created {len(chunks)} chunks from new content")

            # Generate embeddings
            chunks_with_embeddings = self.embedder.batch_process_chunks(chunks)
            print("Embeddings generated for new content")

            # Store in vector database
            success = self._store_in_vector_db(chunks_with_embeddings)
            return success
        except Exception as e:
            logging.error(f"Error updating content: {e}")
            return False


# Example usage
if __name__ == "__main__":
    pipeline = IngestionPipeline()
    success = pipeline.run_full_ingestion()
    if success:
        print("Ingestion pipeline completed successfully!")
    else:
        print("Ingestion pipeline failed!")