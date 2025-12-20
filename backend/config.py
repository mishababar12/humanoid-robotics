from pydantic_settings import BaseSettings
from typing import Optional, List


class Settings(BaseSettings):
    # API Keys and External Services
    cohere_api_key: str
    qdrant_url: str
    qdrant_api_key: Optional[str] = None
    database_url: str
    neon_db_url: str
    redis_url: Optional[str] = "redis://localhost:6379/0"

    # Application Settings
    debug: bool = False
    app_name: str = "RAG Chatbot API"
    version: str = "1.0.0"
    api_prefix: str = "/api"

    # CORS Settings
    allowed_origins: str = "*"  # Comma-separated list in production

    @property
    def cors_origins(self) -> List[str]:
        """Parse comma-separated origins into list"""
        if self.allowed_origins == "*":
            return ["*"]
        return [origin.strip() for origin in self.allowed_origins.split(",")]

    # Qdrant Configuration
    qdrant_collection_name: str = "book_content"

    # Cohere Configuration
    cohere_embedding_model: str = "embed-english-v3.0"
    cohere_generation_model: str = "command-nightly"

    # Performance Settings
    max_concurrent_requests: int = 100
    response_timeout: int = 30

    # Content Processing
    chunk_size: int = 512
    chunk_overlap: int = 50
    max_search_results: int = 5

    class Config:
        env_file = ".env"


settings = Settings()