#!/usr/bin/env python3
"""
Test script to verify database connection and session creation
"""
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from config import settings
from database.postgres import engine, Base
from services.session_service import session_service

def test_db_connection():
    print(f"Database URL: {settings.database_url}")

    try:
        # Test basic connection
        from sqlalchemy import text
        with engine.connect() as conn:
            result = conn.execute(text("SELECT 1"))
            print("[OK] Basic database connection successful")
    except Exception as e:
        print(f"[ERROR] Basic database connection failed: {e}")
        return False

    try:
        # Create tables
        Base.metadata.create_all(bind=engine)
        print("[OK] Database tables created successfully")
    except Exception as e:
        print(f"[ERROR] Table creation failed: {e}")
        return False

    try:
        # Test session creation
        session_id = session_service.create_session()
        if session_id:
            print(f"[OK] Session created successfully: {session_id}")
        else:
            print("[ERROR] Session creation failed - returned empty ID")
            return False
    except Exception as e:
        print(f"[ERROR] Session creation failed: {e}")
        return False

    return True

if __name__ == "__main__":
    print("Testing database connection and session creation...")
    success = test_db_connection()
    if success:
        print("\n[SUCCESS] All tests passed!")
    else:
        print("\n[FAILURE] Some tests failed!")
        sys.exit(1)