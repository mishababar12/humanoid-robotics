# RAG Chatbot Backend

This is the backend service for the Retrieval-Augmented Generation (RAG) chatbot that integrates with the Docusaurus-deployed book.

## Architecture

The backend is built with FastAPI and includes:

- **Content Ingestion**: Extracts content from Docusaurus docs, chunks it, and generates embeddings
- **Vector Database**: Uses Qdrant Cloud for efficient semantic search
- **AI Integration**: Uses Cohere for embeddings and response generation
- **Session Management**: Tracks conversation history and user sessions
- **API Layer**: Provides RESTful endpoints for the frontend

## Components

### Ingestion Pipeline
- `ingestion/extractor.py`: Extracts content from Docusaurus markdown files
- `ingestion/chunker.py`: Chunks content into semantic boundaries
- `ingestion/embedder.py`: Generates embeddings using Cohere API
- `ingestion/pipeline.py`: Orchestrates the full ingestion process

### Services
- `services/search_service.py`: Handles semantic search functionality
- `services/generation_service.py`: Generates responses using Cohere
- `services/context_service.py`: Manages text selection and conversation context
- `services/session_service.py`: Manages user sessions and history

### Database Layer
- `database/postgres.py`: PostgreSQL integration for session management
- `database/qdrant.py`: Vector database operations

### API Layer
- `app/api/chat.py`: Main chat and search endpoints
- `app/api/health.py`: Health check endpoints
- `app/main.py`: FastAPI application entry point

## Setup

1. Install dependencies:
```bash
pip install -r requirements.txt
```

2. Set up environment variables:
```bash
cp .env.example .env
# Edit .env with your API keys and configuration
```

3. Run the content indexing pipeline:
```bash
python scripts/index_content.py
```

4. Start the server:
```bash
python -m app.main
```

## API Endpoints

- `POST /api/chat` - Main chat endpoint
- `POST /api/search` - Standalone search endpoint
- `GET /api/health` - Health check
- `GET /api/ready` - Readiness check
- `POST /api/session` - Create session
- `DELETE /api/session/{session_id}` - Clear session

## Environment Variables

- `COHERE_API_KEY`: Your Cohere API key
- `QDRANT_URL`: URL for your Qdrant Cloud instance
- `QDRANT_API_KEY`: API key for Qdrant Cloud (if required)
- `DATABASE_URL`: Connection string for PostgreSQL database
- `NEON_DB_URL`: Connection string for Neon Postgres (if different)
- `REDIS_URL`: Redis connection string (optional)
- `DEBUG`: Set to "true" for debug mode

## Running the Service

```bash
# Install dependencies
pip install -r requirements.txt

# Index the book content
python scripts/index_content.py

# Start the API server
python -m app.main
```

The API will be available at `http://localhost:8000` with documentation at `http://localhost:8000/docs`.

## Testing

Run the indexing script to load your book content into the vector database, then test the API endpoints. The chat endpoint expects:

```json
{
  "message": "Your question here",
  "selected_text": "Optional selected text for context",
  "session_id": "Optional session ID",
  "context": "general or selected_text"
}
```