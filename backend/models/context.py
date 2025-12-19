from typing import Dict, List, Optional


class ContextData:
    def __init__(self, session_id: str):
        self.session_id = session_id
        self.selected_text: Optional[str] = None
        self.source_info: Dict = {}
        self.has_selection: bool = False
        self.conversation_history: List[Dict] = {}