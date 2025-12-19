#!/usr/bin/env python3
"""
Script to run the content indexing pipeline
This script will extract content from Docusaurus docs, chunk it,
generate embeddings, and store in the vector database.
"""

import sys
import os
from pathlib import Path

# Add the backend directory to the path so we can import our modules
backend_dir = Path(__file__).parent.parent
sys.path.insert(0, str(backend_dir))

# Add backend to path for relative imports
if str(backend_dir) not in sys.path:
    sys.path.insert(0, str(backend_dir))

from ingestion.pipeline import IngestionPipeline
from config import settings
import logging


def main():
    print("Starting RAG Chatbot Content Indexing Pipeline...")
    print(f"Configuration:")
    print(f"  - Docs path: ../my-website/docs")
    print(f"  - Qdrant collection: {settings.qdrant_collection_name}")
    print(f"  - Cohere model: {settings.cohere_embedding_model}")
    print(f"  - Chunk size: {settings.chunk_size}")
    print(f"  - Chunk overlap: {settings.chunk_overlap}")
    print()

    try:
        # Initialize the ingestion pipeline
        # Use relative path from backend directory to the docs
        import os
        docs_path = os.path.join(backend_dir, "..", "my-website", "docs")
        pipeline = IngestionPipeline(docs_path=docs_path)

        print("Running ingestion pipeline...")
        success = pipeline.run_full_ingestion()

        if success:
            print()
            print("✅ Content indexing completed successfully!")

            # Show collection info
            from database.qdrant import qdrant_service
            info = qdrant_service.get_collection_info()
            if info:
                print(f"📊 Collection info:")
                print(f"   - Points stored: {info.get('points_count', 'Unknown')}")
                print(f"   - Vector size: {info.get('vector_size', 'Unknown')}")
        else:
            print()
            print("❌ Content indexing failed!")
            sys.exit(1)

    except Exception as e:
        print(f"❌ Error during content indexing: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()