#!/usr/bin/env python3
"""
Custom script to ingest Docusaurus documentation into Qdrant with smaller batch sizes
"""
import sys
from pathlib import Path

# Add backend directory to path for imports
backend_dir = Path(__file__).parent.parent
if str(backend_dir) not in sys.path:
    sys.path.insert(0, str(backend_dir))

from ingestion.pipeline import IngestionPipeline
from ingestion.extractor import ContentExtractor
from ingestion.chunker import ContentChunker
from ingestion.embedder import EmbedderService
from database.qdrant import qdrant_service
from config import settings
import time
import uuid
from qdrant_client.http import models
from typing import List, Dict


def custom_store_in_vector_db(chunks: List[Dict], batch_size: int = 10) -> bool:
    """
    Store processed chunks in the vector database with smaller batch sizes
    """
    if not qdrant_service.connected or not qdrant_service.client:
        print("Error: Cannot store chunks - Qdrant is not connected")
        return False

    try:
        print(f"Starting to store {len(chunks)} chunks with batch size {batch_size}")

        # Process in smaller batches to avoid timeouts
        for i in range(0, len(chunks), batch_size):
            batch = chunks[i:i + batch_size]
            points = []

            for chunk in batch:
                # Use a proper UUID string as the ID
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

            print(f"Uploading batch {i//batch_size + 1}/{(len(chunks)-1)//batch_size + 1} with {len(points)} points...")

            # Add timeout handling
            try:
                qdrant_service.client.upsert(
                    collection_name=qdrant_service.collection_name,
                    points=points
                )
                print(f"Successfully uploaded batch {i//batch_size + 1}")
            except Exception as batch_error:
                print(f"Error uploading batch {i//batch_size + 1}: {batch_error}")
                print("Retrying after 5 seconds...")
                time.sleep(5)
                try:
                    qdrant_service.client.upsert(
                        collection_name=qdrant_service.collection_name,
                        points=points
                    )
                    print(f"Successfully uploaded batch {i//batch_size + 1} on retry")
                except Exception as retry_error:
                    print(f"Retry failed for batch {i//batch_size + 1}: {retry_error}")
                    return False

        print("All batches uploaded successfully!")
        return True
    except Exception as e:
        print(f"Error storing chunks in vector database: {e}")
        import traceback
        traceback.print_exc()
        return False


def run_ingestion():
    """
    Run the full ingestion process with custom batch handling
    """
    try:
        print("Starting content extraction...")
        extractor = ContentExtractor(docs_path="../my-website/docs")
        content = extractor.extract_all_content()
        print(f"Extracted {len(content)} content sections")

        if not content:
            print("No content found to process")
            return False

        print("Chunking content...")
        chunker = ContentChunker(
            chunk_size=settings.chunk_size,
            chunk_overlap=settings.chunk_overlap
        )
        chunks = chunker.chunk_content(content)
        print(f"Created {len(chunks)} content chunks")

        print("Generating embeddings...")
        embedder = EmbedderService()
        chunks_with_embeddings = embedder.batch_process_chunks(chunks)
        print("Embeddings generated successfully")

        print("Storing in vector database with smaller batches...")
        success = custom_store_in_vector_db(chunks_with_embeddings, batch_size=5)
        if success:
            print("Content successfully stored in vector database")
        else:
            print("Failed to store content in vector database")
            return False

        return True
    except Exception as e:
        print(f"Error in ingestion: {e}")
        import traceback
        traceback.print_exc()
        return False


if __name__ == "__main__":
    print("Starting custom ingestion of Docusaurus documentation...")
    success = run_ingestion()

    if success:
        print("\nIngestion pipeline completed successfully!")
        print("Your MDX book content has been indexed into Qdrant!")
    else:
        print("\nIngestion pipeline failed!")
        print("The content may not be fully indexed in Qdrant.")