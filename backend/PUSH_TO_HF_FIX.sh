#!/bin/bash

# Fix: Push ONLY backend folder to HF Spaces
# This ensures Dockerfile is at root level of HF Space

echo "🚀 Pushing Backend to Hugging Face Spaces..."
echo ""

# Backend folder mein already hain
cd "$(dirname "$0")"

echo "✓ Current directory: $(pwd)"
echo ""

# Check if Dockerfile exists
if [ ! -f "Dockerfile" ]; then
    echo "❌ Error: Dockerfile not found!"
    echo "Make sure you're in the backend folder"
    exit 1
fi

echo "✓ Dockerfile found"
echo ""

# Check if HF remote exists
if git remote get-url hf > /dev/null 2>&1; then
    echo "✓ HF remote already configured"
    git remote get-url hf
else
    echo "❌ HF remote not found. Add it first:"
    echo "git remote add hf https://huggingface.co/spaces/YOUR_USERNAME/rag-chatbot-backend"
    exit 1
fi

echo ""
echo "📦 Adding all files..."
git add .

echo ""
echo "💾 Creating commit..."
git commit -m "Deploy backend to HF Spaces" || echo "No changes to commit"

echo ""
echo "🚀 Pushing to HF Spaces..."
git push hf HEAD:main -f

echo ""
echo "✅ Push complete!"
echo ""
echo "Now check your HF Space:"
echo "https://huggingface.co/spaces/Mishababar/rag-chatbot-backend"
echo ""
echo "Go to Logs tab to see build progress"
