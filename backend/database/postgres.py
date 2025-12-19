from sqlalchemy import create_engine, Column, Integer, String, DateTime, Text, ForeignKey
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker, relationship
from sqlalchemy.sql import func
from config import settings
from typing import Optional
import uuid
from datetime import datetime
from pydantic import BaseModel
import logging
import time
from contextlib import contextmanager


# Database setup with connection pooling and retry logic
def create_engine_with_retry():
    from sqlalchemy import text
    max_retries = 3
    retry_delay = 1

    for attempt in range(max_retries):
        try:
            # Configure engine with connection pooling for Neon
            engine = create_engine(
                settings.database_url,
                pool_size=5,
                max_overflow=10,
                pool_pre_ping=True,  # Verify connections before use
                pool_recycle=300,    # Recycle connections every 5 minutes
                echo=False,          # Set to True for debugging
                connect_args={
                    "connect_timeout": 10,
                    "application_name": "rag_chatbot"
                }
            )
            # Test the connection
            with engine.connect() as conn:
                conn.execute(text("SELECT 1"))
            logging.info("Database connection established successfully")
            return engine
        except Exception as e:
            logging.error(f"Database connection attempt {attempt + 1} failed: {e}")
            if attempt == max_retries - 1:
                logging.error("All database connection attempts failed")
                raise e
            time.sleep(retry_delay * (2 ** attempt))  # Exponential backoff


def get_db_with_retry():
    """Generator that yields a database session with retry logic"""
    from sqlalchemy import text
    max_retries = 3
    retry_delay = 0.5

    for attempt in range(max_retries):
        db = None
        try:
            db = SessionLocal()
            # Test the session
            db.execute(text("SELECT 1"))
            yield db
            return  # Success, exit the retry loop
        except Exception as e:
            logging.error(f"Database session attempt {attempt + 1} failed: {e}")
            if db:
                db.close()
            if attempt == max_retries - 1:
                raise e
            time.sleep(retry_delay)
        finally:
            if db:
                db.close()


# Create engine with retry logic
engine = create_engine_with_retry()
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)
Base = declarative_base()


# Database Models
class UserSession(Base):
    __tablename__ = "user_sessions"

    id = Column(String, primary_key=True, default=lambda: str(uuid.uuid4()))
    created_at = Column(DateTime, default=func.now())
    updated_at = Column(DateTime, default=func.now(), onupdate=func.now())
    last_activity = Column(DateTime, default=func.now())
    session_metadata = Column(Text)  # JSON metadata for session


class ChatMessage(Base):
    __tablename__ = "chat_messages"

    id = Column(Integer, primary_key=True, index=True)
    session_id = Column(String, ForeignKey("user_sessions.id"))
    role = Column(String)  # "user" or "assistant"
    content = Column(Text)
    timestamp = Column(DateTime, default=func.now())
    source_chunks = Column(Text)  # JSON of source chunks used

    session = relationship("UserSession", back_populates="messages")


UserSession.messages = relationship("ChatMessage", back_populates="session")


# Pydantic models for API
class UserSessionCreate(BaseModel):
    metadata: Optional[dict] = None


class UserSessionResponse(BaseModel):
    id: str
    created_at: datetime
    updated_at: datetime
    last_activity: datetime
    session_metadata: Optional[dict] = None

    class Config:
        from_attributes = True


class ChatMessageCreate(BaseModel):
    session_id: str
    role: str
    content: str
    source_chunks: Optional[list] = None


class ChatMessageResponse(BaseModel):
    id: int
    session_id: str
    role: str
    content: str
    timestamp: datetime
    source_chunks: Optional[list] = None

    class Config:
        from_attributes = True


# Dependency to get DB session
def get_db():
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()