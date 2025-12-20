---
title: RAG Chatbot Backend
emoji: 🤖
colorFrom: blue
colorTo: purple
sdk: docker
app_port: 7860
pinned: false
---

# RAG Chatbot Backend API

This is the backend service for the Retrieval-Augmented Generation (RAG) chatbot that integrates with the Docusaurus-deployed book.

## Features

- **FastAPI** backend with automatic API documentation
- **Cohere** for embeddings and text generation
- **Qdrant Cloud** for vector search
- **Neon Postgres** for session management
- **Docker** containerized deployment

## API Endpoints

- `GET /health` - Health check
- `GET /docs` - Interactive API documentation
- `POST /api/chat` - Chat with the AI assistant
- `POST /api/search` - Search through book content

## Environment Variables

This Space requires the following secrets to be configured:

- `COHERE_API_KEY` - Your Cohere API key
- `QDRANT_URL` - Qdrant cluster URL
- `QDRANT_API_KEY` - Qdrant API key
- `NEON_DB_URL` - Neon Postgres connection string
- `DATABASE_URL` - Same as NEON_DB_URL
- `DEBUG` - Set to "false" for production
- `ALLOWED_ORIGINS` - CORS allowed origins (e.g., https://your-site.github.io)

## Usage

Once deployed, the API will be available at:
- Base URL: `https://your-username-space-name.hf.space`
- API Docs: `https://your-username-space-name.hf.space/docs`

## Tech Stack

- Python 3.10
- FastAPI + Uvicorn
- Cohere AI
- Qdrant Vector Database
- PostgreSQL (Neon)
- Docker

## License

Apache 2.0
