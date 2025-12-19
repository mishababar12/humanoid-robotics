from typing import Dict, Optional, List
from datetime import datetime, timedelta
from database.postgres import UserSession, ChatMessage, get_db_with_retry
from sqlalchemy.orm import Session
from sqlalchemy import and_
import uuid
import logging


class SessionService:
    """
    Service to handle user sessions and conversation history
    """

    def create_session(self, metadata: Optional[Dict] = None) -> str:
        """
        Create a new user session
        """
        for db in get_db_with_retry():
            try:
                session = UserSession(
                    id=str(uuid.uuid4()),
                    session_metadata=str(metadata) if metadata else None
                )
                db.add(session)
                db.commit()
                db.refresh(session)
                logging.info(f"Session created successfully: {session.id}")
                return session.id
            except Exception as e:
                db.rollback()
                logging.error(f"Error creating session: {e}")
                raise e
            finally:
                db.close()
        return ""

    def get_session(self, session_id: str) -> Optional[UserSession]:
        """
        Get session by ID
        """
        for db in get_db_with_retry():
            try:
                session = db.query(UserSession).filter(UserSession.id == session_id).first()
                return session
            except Exception as e:
                logging.error(f"Error getting session: {e}")
                raise e
            finally:
                db.close()
        return None

    def add_message_to_session(self, session_id: str, role: str, content: str, source_chunks: Optional[List] = None) -> bool:
        """
        Add a message to a session's conversation history
        """
        for db in get_db_with_retry():
            try:
                message = ChatMessage(
                    session_id=session_id,
                    role=role,
                    content=content,
                    source_chunks=str(source_chunks) if source_chunks else None
                )
                db.add(message)
                db.commit()
                logging.info(f"Message added to session {session_id}")
                return True
            except Exception as e:
                db.rollback()
                logging.error(f"Error adding message to session: {e}")
                raise e
            finally:
                db.close()
        return False

    def get_conversation_history(self, session_id: str, limit: int = 10) -> List[ChatMessage]:
        """
        Get conversation history for a session
        """
        for db in get_db_with_retry():
            try:
                messages = db.query(ChatMessage)\
                    .filter(ChatMessage.session_id == session_id)\
                    .order_by(ChatMessage.timestamp.desc())\
                    .limit(limit)\
                    .all()
                result = messages[::-1]  # Reverse to get chronological order
                logging.info(f"Retrieved {len(result)} messages for session {session_id}")
                return result
            except Exception as e:
                logging.error(f"Error getting conversation history: {e}")
                raise e
            finally:
                db.close()
        return []

    def clear_session_history(self, session_id: str) -> bool:
        """
        Clear conversation history for a session
        """
        for db in get_db_with_retry():
            try:
                db.query(ChatMessage).filter(ChatMessage.session_id == session_id).delete()
                db.commit()
                logging.info(f"Session history cleared for session {session_id}")
                return True
            except Exception as e:
                db.rollback()
                logging.error(f"Error clearing session history: {e}")
                raise e
            finally:
                db.close()
        return False

    def cleanup_old_sessions(self, days_old: int = 30) -> bool:
        """
        Remove sessions older than specified number of days
        """
        for db in get_db_with_retry():
            try:
                cutoff_date = datetime.now() - timedelta(days=days_old)
                old_sessions = db.query(UserSession)\
                    .filter(UserSession.last_activity < cutoff_date)\
                    .all()

                for session in old_sessions:
                    # Delete associated messages first due to foreign key constraint
                    db.query(ChatMessage).filter(ChatMessage.session_id == session.id).delete()
                    db.delete(session)

                db.commit()
                logging.info(f"Cleaned up {len(old_sessions)} old sessions")
                return True
            except Exception as e:
                db.rollback()
                logging.error(f"Error cleaning up old sessions: {e}")
                raise e
            finally:
                db.close()
        return False

    def update_session_activity(self, session_id: str) -> bool:
        """
        Update the last activity timestamp for a session
        """
        for db in get_db_with_retry():
            try:
                session = db.query(UserSession).filter(UserSession.id == session_id).first()
                if session:
                    session.last_activity = datetime.now()
                    db.commit()
                    return True
                return False
            except Exception as e:
                db.rollback()
                logging.error(f"Error updating session activity: {e}")
                raise e
            finally:
                db.close()
        return False


# Global instance
session_service = SessionService()