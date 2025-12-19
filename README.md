# Physical AI & Humanoid Robotics - RAG Chatbot

This project implements an integrated RAG (Retrieval-Augmented Generation) chatbot for the Physical AI & Humanoid Robotics textbook. The system allows users to ask questions about the book content and get answers with proper citations.

## Architecture

The system consists of two main components:

1. **Backend API** - FastAPI server with RAG functionality
2. **Frontend Website** - Docusaurus documentation site with integrated chatbot

## Technology Stack

- **Backend**: FastAPI, Python
- **Database**: Neon Serverless Postgres, Qdrant Cloud
- **AI Services**: Cohere for embeddings and generation (using updated models)
- **Frontend**: Docusaurus, React, TypeScript
- **RAG Pipeline**: Content extraction, chunking, embedding, retrieval

## Features

- **Content-based Q&A**: Ask questions about the textbook content
- **Selected Text Focus**: Ask questions specifically about selected text
- **Source Citations**: Responses include references to original content
- **Session Management**: Maintain conversation history
- **Responsive UI**: Works on desktop and mobile devices

## Setup and Installation

### Prerequisites

- Python 3.8+
- Node.js 16+
- Access to Cohere API
- Qdrant Cloud account
- Neon Postgres account

### Backend Setup

1. Navigate to the backend directory:
   ```bash
   cd backend
   ```

2. Install Python dependencies:
   ```bash
   pip install -r requirements.txt
   ```

3. Configure environment variables in `.env`:
   ```env
   COHERE_API_KEY=your_cohere_api_key
   QDRANT_URL=your_qdrant_url
   QDRANT_API_KEY=your_qdrant_api_key
   DATABASE_URL=your_neon_postgres_url
   NEON_DB_URL=your_neon_postgres_url
   ```

4. Start the backend server:
   ```bash
   python -m app.main
   ```
   The API will be available at `http://localhost:8000`

### Frontend Setup

1. Navigate to the website directory:
   ```bash
   cd my-website
   ```

2. Install dependencies:
   ```bash
   npm install
   ```

3. Start the development server:
   ```bash
   BACKEND_URL=http://localhost:8000 npx docusaurus start
   ```
   The website will be available at `http://localhost:3000`

### Content Indexing

Before the chatbot can answer questions with rich context from your book, you need to index your book content:

1. Make sure the backend server is running
2. Run the indexing script:
   ```bash
   cd backend
   python scripts/index_content.py
   ```

This will extract content from the `my-website/docs` directory, chunk it, generate embeddings, and store it in Qdrant.

**Note**: If Qdrant is not properly configured or if content indexing fails, the chatbot will still function but will provide more general responses without specific citations from your book content. The error message "I encountered an error while generating a response" typically appears when Qdrant is not accessible.

**To ensure full functionality**:
1. Verify your Qdrant Cloud credentials in the `.env` file
2. Run the content indexing script successfully
3. Confirm that 400+ content sections are processed during indexing
4. Note: The system uses updated Cohere models (command-nightly) due to API changes

## Usage

1. Start both the backend and frontend servers
2. Navigate to the documentation website
3. Use the chatbot widget (bottom-right corner) to ask questions
4. Select text on any page to ask specific questions about that content

## API Endpoints

- `GET /` - Health check and info
- `GET /health` - System health status
- `POST /api/chat` - Main chat functionality
- `POST /api/search` - Standalone search
- `POST /api/session` - Create new session
- `DELETE /api/session/{id}` - Clear session
- `GET /api/session/{id}/history` - Get conversation history
- `POST /api/feedback` - Submit feedback

## Deployment

### Backend Deployment

Deploy the FastAPI backend to any cloud platform (AWS, GCP, Azure, Vercel, etc.) that supports Python applications.

### Frontend Deployment

The Docusaurus site can be deployed to:
- GitHub Pages
- Vercel
- Netlify
- Any static hosting service

Update the `BACKEND_URL` in your deployment environment to point to your deployed backend.

## Troubleshooting

- If the chatbot doesn't appear, ensure both servers are running
- Check browser console for any JavaScript errors
- Verify API endpoints are accessible from the frontend
- Ensure all environment variables are properly configured

## Development

For development, run both servers simultaneously:
- Backend: `cd backend && python -m app.main`
- Frontend: `cd my-website && BACKEND_URL=http://localhost:8000 npx docusaurus start`

## License

This project is part of the Physical AI & Humanoid Robotics textbook materials.