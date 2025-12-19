import sys
from pathlib import Path
from typing import Dict, Optional, List

# Add backend directory to path for imports
backend_dir = Path(__file__).parent.parent
if str(backend_dir) not in sys.path:
    sys.path.insert(0, str(backend_dir))

from models.context import ContextData


class ContextService:
    """
    Service to handle text selection context and conversation context
    """

    def __init__(self):
        # In a real implementation, this might use Redis or database for persistence
        self.context_store: Dict[str, ContextData] = {}

    def set_selection_context(self, session_id: str, selected_text: str, source_info: Optional[Dict] = None) -> bool:
        """
        Set the selected text context for a session
        """
        try:
            if session_id not in self.context_store:
                self.context_store[session_id] = ContextData(session_id=session_id)

            self.context_store[session_id].selected_text = selected_text
            self.context_store[session_id].source_info = source_info or {}
            self.context_store[session_id].has_selection = True
            return True
        except Exception as e:
            print(f"Error setting selection context: {e}")
            return False

    def get_selection_context(self, session_id: str) -> Optional[str]:
        """
        Get the selected text context for a session
        """
        context_data = self.context_store.get(session_id)
        if context_data and context_data.has_selection:
            return context_data.selected_text
        return None

    def clear_selection_context(self, session_id: str) -> bool:
        """
        Clear the selected text context for a session
        """
        try:
            if session_id in self.context_store:
                self.context_store[session_id].selected_text = None
                self.context_store[session_id].source_info = {}
                self.context_store[session_id].has_selection = False
            return True
        except Exception as e:
            print(f"Error clearing selection context: {e}")
            return False

    def set_conversation_context(self, session_id: str, conversation_history: List[Dict]) -> bool:
        """
        Set the conversation history context for a session
        """
        try:
            if session_id not in self.context_store:
                self.context_store[session_id] = ContextData(session_id=session_id)

            self.context_store[session_id].conversation_history = conversation_history
            return True
        except Exception as e:
            print(f"Error setting conversation context: {e}")
            return False

    def get_conversation_context(self, session_id: str) -> List[Dict]:
        """
        Get the conversation history context for a session
        """
        context_data = self.context_store.get(session_id)
        if context_data:
            return context_data.conversation_history
        return []

    def add_to_conversation(self, session_id: str, message: Dict) -> bool:
        """
        Add a message to the conversation history for a session
        """
        try:
            if session_id not in self.context_store:
                self.context_store[session_id] = ContextData(session_id=session_id)

            self.context_store[session_id].conversation_history.append(message)
            # Limit history to prevent memory issues
            if len(self.context_store[session_id].conversation_history) > 20:
                self.context_store[session_id].conversation_history = \
                    self.context_store[session_id].conversation_history[-10:]
            return True
        except Exception as e:
            print(f"Error adding to conversation: {e}")
            return False

    def get_context_for_session(self, session_id: str) -> Optional[ContextData]:
        """
        Get full context data for a session
        """
        return self.context_store.get(session_id)

    def clear_session_context(self, session_id: str) -> bool:
        """
        Clear all context for a session
        """
        try:
            if session_id in self.context_store:
                del self.context_store[session_id]
            return True
        except Exception as e:
            print(f"Error clearing session context: {e}")
            return False


# Example model class (this would typically be in models/context.py)
class ContextData:
    def __init__(self, session_id: str):
        self.session_id = session_id
        self.selected_text: Optional[str] = None
        self.source_info: Dict = {}
        self.has_selection: bool = False
        self.conversation_history: List[Dict] = []


# Global instance
context_service = ContextService()