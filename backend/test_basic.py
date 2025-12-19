"""
Basic test file to verify the RAG chatbot backend components are properly set up
"""
import sys
import os
from pathlib import Path

# Add the project root directory to the path so we can import our modules
project_root = Path(__file__).parent.parent
sys.path.insert(0, str(project_root))

# Change to the backend directory for proper relative imports
original_cwd = os.getcwd()
os.chdir(str(Path(__file__).parent))

def test_imports():
    """Test that all modules can be imported without errors"""
    print("Testing imports...")

    try:
        from config import settings
        print("[OK] Config module imported successfully")
    except Exception as e:
        print(f"[ERROR] Error importing config: {e}")
        return False

    try:
        from database.postgres import UserSession, ChatMessage, get_db
        print("[OK] Postgres database module imported successfully")
    except Exception as e:
        print(f"[ERROR] Error importing postgres: {e}")
        return False

    try:
        from database.qdrant import qdrant_service
        print("[OK] Qdrant database module imported successfully")
    except Exception as e:
        print(f"[ERROR] Error importing qdrant: {e}")
        return False

    try:
        from ingestion.extractor import ContentExtractor
        print("[OK] Content extractor module imported successfully")
    except Exception as e:
        print(f"[ERROR] Error importing extractor: {e}")
        return False

    try:
        from ingestion.chunker import ContentChunker
        print("[OK] Content chunker module imported successfully")
    except Exception as e:
        print(f"[ERROR] Error importing chunker: {e}")
        return False

    try:
        from ingestion.embedder import EmbedderService
        print("[OK] Embedder service module imported successfully")
    except Exception as e:
        print(f"[ERROR] Error importing embedder: {e}")
        return False

    try:
        from ingestion.pipeline import IngestionPipeline
        print("[OK] Ingestion pipeline module imported successfully")
    except Exception as e:
        print(f"[ERROR] Error importing pipeline: {e}")
        return False

    try:
        from services.search_service import search_service
        print("[OK] Search service module imported successfully")
    except Exception as e:
        print(f"[ERROR] Error importing search service: {e}")
        return False

    try:
        from services.generation_service import generation_service
        print("[OK] Generation service module imported successfully")
    except Exception as e:
        print(f"[ERROR] Error importing generation service: {e}")
        return False

    try:
        from services.context_service import context_service
        print("[OK] Context service module imported successfully")
    except Exception as e:
        print(f"[ERROR] Error importing context service: {e}")
        return False

    try:
        from services.session_service import session_service
        print("[OK] Session service module imported successfully")
    except Exception as e:
        print(f"[ERROR] Error importing session service: {e}")
        return False

    try:
        from app.main import app
        print("[OK] FastAPI app module imported successfully")
    except Exception as e:
        print(f"[ERROR] Error importing FastAPI app: {e}")
        return False

    return True


def test_config():
    """Test that configuration is properly loaded"""
    print("\nTesting configuration...")

    try:
        from config import settings

        # Check that required settings exist (without printing sensitive values)
        required_attrs = [
            'cohere_api_key', 'qdrant_url', 'database_url',
            'neon_db_url', 'qdrant_collection_name'
        ]

        for attr in required_attrs:
            if not hasattr(settings, attr):
                print(f"[ERROR] Missing required config attribute: {attr}")
                return False

        print("[OK] Configuration loaded successfully")
        return True
    except Exception as e:
        print(f"[ERROR] Error testing configuration: {e}")
        return False


def test_services():
    """Test that service instances are created properly"""
    print("\nTesting services...")

    try:
        # Import services with proper relative paths from backend root
        import sys
        from pathlib import Path
        # Add the backend directory to path for imports
        backend_dir = Path(__file__).parent
        if str(backend_dir) not in sys.path:
            sys.path.insert(0, str(backend_dir))

        from services.search_service import search_service
        from services.generation_service import generation_service
        from services.context_service import context_service
        from services.session_service import session_service
        from database.qdrant import qdrant_service

        services = [
            ("Search Service", search_service),
            ("Generation Service", generation_service),
            ("Context Service", context_service),
            ("Session Service", session_service),
            ("Qdrant Service", qdrant_service)
        ]

        for name, service in services:
            if service is None:
                print(f"[ERROR] {name} not properly initialized")
                return False
            print(f"[OK] {name} initialized successfully")

        return True
    except Exception as e:
        print(f"[ERROR] Error testing services: {e}")
        import traceback
        traceback.print_exc()
        return False


def main():
    print("Running basic tests for RAG Chatbot Backend...")
    print("="*50)

    all_tests_passed = True

    # Run all tests
    all_tests_passed &= test_imports()
    all_tests_passed &= test_config()
    all_tests_passed &= test_services()

    print("\n" + "="*50)
    if all_tests_passed:
        print("[SUCCESS] All tests passed! The RAG chatbot backend is properly set up.")
        print("\nNext steps:")
        print("1. Set up your environment variables in .env")
        print("2. Run the content indexing pipeline: python scripts/index_content.py")
        print("3. Start the API server: python -m app.main")
    else:
        print("[ERROR] Some tests failed. Please check the errors above.")
        return 1

    return 0


if __name__ == "__main__":
    try:
        exit(main())
    finally:
        # Restore original working directory
        os.chdir(original_cwd)