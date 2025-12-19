from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from app.api import chat, health
import sys
from pathlib import Path

# Add backend directory to path for imports
backend_dir = Path(__file__).parent.parent
if str(backend_dir) not in sys.path:
    sys.path.insert(0, str(backend_dir))

from config import settings
from database.postgres import Base, engine
import uvicorn
import logging


# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Create database tables
try:
    Base.metadata.create_all(bind=engine)
    logger.info("Database tables created successfully")
except Exception as e:
    logger.error(f"Error creating database tables: {e}")


# Create FastAPI app instance
app = FastAPI(
    title=settings.app_name,
    description="RAG Chatbot API for Docusaurus Book Integration",
    version=settings.version,
    docs_url="/docs",
    redoc_url="/redoc"
)


# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


# Include API routes
app.include_router(chat.router, prefix=settings.api_prefix, tags=["chat"])
app.include_router(health.router, prefix=settings.api_prefix, tags=["health"])


# Root endpoint
@app.get("/")
async def root():
    return {
        "message": "RAG Chatbot API",
        "version": settings.version,
        "status": "running",
        "docs": "/docs for API documentation"
    }


# Health check at root level as well
@app.get("/health")
async def root_health():
    return {"status": "healthy", "service": "RAG Chatbot API"}


if __name__ == "__main__":
    logger.info(f"Starting {settings.app_name} v{settings.version}")
    logger.info(f"API available at: http://localhost:8000")
    logger.info(f"Documentation available at: http://localhost:8000/docs")

    uvicorn.run(
        "app.main:app",
        host="0.0.0.0",
        port=8000,
        reload=True if settings.debug else False,
        log_level="info" if settings.debug else "error"
    )