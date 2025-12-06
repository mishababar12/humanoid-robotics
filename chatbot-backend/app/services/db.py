import psycopg2
import os

# This is a conceptual example of how to connect to a Neon Serverless Postgres database.
# In a real application, you would use a more robust connection management system,
# and you would get the database credentials from environment variables.

def get_db_connection():
    """
    Establishes a connection to the Neon Serverless Postgres database.
    """
    try:
        conn = psycopg2.connect(
            host=os.environ.get("DB_HOST"),
            database=os.environ.get("DB_NAME"),
            user=os.environ.get("DB_USER"),
            password=os.environ.get("DB_PASSWORD"),
            port=os.environ.get("DB_PORT"),
            sslmode='require'
        )
        return conn
    except psycopg2.Error as e:
        print(f"Error connecting to database: {e}")
        return None

def initialize_db():
    """
    Initializes the database by creating the necessary tables.
    """
    conn = get_db_connection()
    if conn:
        try:
            cur = conn.cursor()
            # Create a table to store the textbook content and embeddings
            cur.execute("""
                CREATE TABLE IF NOT EXISTS textbook_content (
                    id SERIAL PRIMARY KEY,
                    module VARCHAR(255),
                    title VARCHAR(255),
                    content TEXT,
                    embedding VECTOR(384)
                );
            """)
            conn.commit()
            cur.close()
            conn.close()
            print("Database initialized successfully.")
        except psycopg2.Error as e:
            print(f"Error initializing database: {e}")
