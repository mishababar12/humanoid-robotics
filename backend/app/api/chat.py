from fastapi import APIRouter, HTTPException, Depends
from pydantic import BaseModel
from typing import List, Optional, Dict
import sys
from pathlib import Path
import ast

# Add backend directory to path for imports
backend_dir = Path(__file__).parent.parent.parent
if str(backend_dir) not in sys.path:
    sys.path.insert(0, str(backend_dir))

from services.search_service import search_service
from services.generation_service import generation_service
from services.context_service import context_service
from services.session_service import session_service
from database.postgres import get_db
from sqlalchemy.orm import Session
import uuid


router = APIRouter()


class ChatRequest(BaseModel):
    message: str
    selected_text: Optional[str] = None
    session_id: Optional[str] = None
    context: Optional[str] = "general"  # "general" or "selected_text"


class ChatResponse(BaseModel):
    response: str
    sources: List[Dict]
    session_id: str
    timestamp: str


@router.post("/chat", response_model=ChatResponse)
async def chat_endpoint(request: ChatRequest):
    """
    Main chat endpoint for the RAG chatbot
    """
    session_id = None
    try:
        # Create or validate session
        if request.session_id:
            session_id = request.session_id
        else:
            # Create a new session in the database
            session_id = session_service.create_session()

        # Store the user message in the session
        session_service.add_message_to_session(
            session_id=session_id,
            role="user",
            content=request.message
        )

        # Update session activity
        session_service.update_session_activity(session_id)

        # Set selection context if provided
        if request.selected_text:
            context_service.set_selection_context(
                session_id=session_id,
                selected_text=request.selected_text
            )

        # Get selection context for this session
        selected_text = context_service.get_selection_context(session_id)

        # Perform search based on message and context
        search_results = search_service.search_content(
            query=request.message,
            selected_text=selected_text if request.context == "selected_text" else None
        )

        # Generate response using the search results
        response_data = generation_service.generate_response_with_citations(
            query=request.message,
            search_results=search_results,
            selected_text=selected_text if request.context == "selected_text" else None
        )

        # Store the AI response in the session
        session_service.add_message_to_session(
            session_id=session_id,
            role="assistant",
            content=response_data["response"],
            source_chunks=response_data.get("detailed_sources", [])
        )

        # Update session activity again after response
        session_service.update_session_activity(session_id)

        # Prepare the response
        from datetime import datetime
        response = ChatResponse(
            response=response_data["response"],
            sources=response_data.get("detailed_sources", [])[:3],  # Limit sources in response
            session_id=session_id,
            timestamp=datetime.now().isoformat()
        )

        return response

    except Exception as e:
        print(f"Error in chat endpoint: {e}")
        # Update session activity even if there's an error
        if session_id:
            try:
                session_service.update_session_activity(session_id)
            except:
                pass  # Ignore errors when updating activity during error handling
        raise HTTPException(status_code=500, detail="Internal server error")


class SearchRequest(BaseModel):
    query: str
    selected_text: Optional[str] = None
    limit: Optional[int] = 5


class SearchResult(BaseModel):
    content: str
    source: str
    title: str
    url: str
    relevance_score: float


class SearchResponse(BaseModel):
    results: List[SearchResult]


@router.post("/search", response_model=SearchResponse)
async def search_endpoint(request: SearchRequest):
    """
    Search endpoint for standalone content search
    """
    try:
        search_results = search_service.search_content(
            query=request.query,
            selected_text=request.selected_text,
            limit=request.limit
        )

        formatted_results = []
        for result in search_results:
            formatted_result = SearchResult(
                content=result.get("content", ""),
                source=result.get("source", ""),
                title=result.get("title", ""),
                url=result.get("url", ""),
                relevance_score=result.get("relevance_score", 0.0)
            )
            formatted_results.append(formatted_result)

        return SearchResponse(results=formatted_results)

    except Exception as e:
        print(f"Error in search endpoint: {e}")
        raise HTTPException(status_code=500, detail="Internal server error")


class SessionRequest(BaseModel):
    metadata: Optional[dict] = None


class SessionResponse(BaseModel):
    session_id: str


@router.post("/session", response_model=SessionResponse)
async def create_session_endpoint(request: SessionRequest = None):
    """
    Create a new user session
    """
    try:
        session_id = session_service.create_session(
            metadata=request.metadata if request else None
        )
        if not session_id:
            raise HTTPException(status_code=500, detail="Failed to create session")

        return SessionResponse(session_id=session_id)
    except HTTPException:
        # Re-raise HTTP exceptions as they are
        raise
    except Exception as e:
        print(f"Error creating session: {e}")
        raise HTTPException(status_code=500, detail="Internal server error")


@router.delete("/session/{session_id}")
async def delete_session_endpoint(session_id: str):
    """
    Clear a user session
    """
    try:
        success = session_service.clear_session_history(session_id)
        if not success:
            raise HTTPException(status_code=500, detail="Failed to clear session")

        # Also clear context for the session
        context_service.clear_session_context(session_id)

        return {"message": "Session cleared successfully"}
    except HTTPException:
        # Re-raise HTTP exceptions as they are
        raise
    except Exception as e:
        print(f"Error clearing session: {e}")
        raise HTTPException(status_code=500, detail="Internal server error")


class Message(BaseModel):
    role: str
    content: str
    timestamp: str
    source_chunks: Optional[List] = None


class ConversationHistoryResponse(BaseModel):
    session_id: str
    messages: List[Message]


@router.get("/session/{session_id}/history", response_model=ConversationHistoryResponse)
async def get_conversation_history_endpoint(session_id: str):
    """
    Get conversation history for a session
    """
    try:
        messages = session_service.get_conversation_history(session_id)
        formatted_messages = []
        for msg in messages:
            formatted_msg = Message(
                role=msg.role,
                content=msg.content,
                timestamp=msg.timestamp.isoformat() if msg.timestamp else "",
                source_chunks=ast.literal_eval(msg.source_chunks) if msg.source_chunks and msg.source_chunks.startswith('[') else None
            )
            formatted_messages.append(formatted_msg)

        return ConversationHistoryResponse(
            session_id=session_id,
            messages=formatted_messages
        )
    except HTTPException:
        # Re-raise HTTP exceptions as they are
        raise
    except Exception as e:
        print(f"Error getting conversation history: {e}")
        raise HTTPException(status_code=500, detail="Internal server error")


class FeedbackRequest(BaseModel):
    session_id: str
    message_id: Optional[str] = None
    rating: int  # 1-5 scale
    feedback_text: Optional[str] = None
    helpful: bool


@router.post("/feedback")
async def feedback_endpoint(request: FeedbackRequest):
    """
    Endpoint to capture user feedback on responses
    """
    try:
        # In a real implementation, you would store this feedback in a database
        # For now, we'll just log it
        print(f"Feedback received: Session {request.session_id}, Rating: {request.rating}, Helpful: {request.helpful}")
        if request.feedback_text:
            print(f"Feedback text: {request.feedback_text}")

        return {"message": "Feedback received successfully"}
    except Exception as e:
        print(f"Error processing feedback: {e}")
        raise HTTPException(status_code=500, detail="Internal server error")