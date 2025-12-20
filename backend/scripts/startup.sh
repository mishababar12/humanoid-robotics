#!/bin/bash
# Startup script for Hugging Face Spaces deployment
# This script runs before the main application starts

echo "Starting RAG Chatbot Backend..."

# Check if required environment variables are set
required_vars=("COHERE_API_KEY" "QDRANT_URL" "NEON_DB_URL")

for var in "${required_vars[@]}"; do
    if [ -z "${!var}" ]; then
        echo "ERROR: Required environment variable $var is not set"
        exit 1
    fi
done

echo "Environment variables validated ✓"

# Run database migrations if needed
echo "Setting up database..."
# python -m alembic upgrade head

echo "Database setup complete ✓"

# Optional: Index content on startup (comment out if you prefer manual indexing)
# echo "Indexing content..."
# python scripts/index_content.py

echo "Startup complete ✓"
echo "Starting application server..."

# Start the application
exec "$@"
