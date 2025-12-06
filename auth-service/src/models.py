from pydantic import BaseModel
from typing import Optional

class UserProfile(BaseModel):
    """
    A Pydantic model for the user profile.
    """
    background: Optional[str] = None # e.g., "software", "hardware", "robotics"

class User(BaseModel):
    """
    A Pydantic model for a user.
    """
    user_id: str
    email: str
    profile: UserProfile