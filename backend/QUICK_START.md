# Quick Start - Deploy Backend to Hugging Face Spaces

## 5-Minute Deployment Checklist

### ✅ Step 1: Get Your API Keys (5 minutes)

1. **Cohere API Key**
   - Go to: https://dashboard.cohere.com
   - Sign up/Login
   - Create API key from dashboard
   - Copy the key

2. **Qdrant Cloud**
   - Go to: https://cloud.qdrant.io
   - Sign up/Login
   - Create a free cluster
   - Copy: Cluster URL and API Key

3. **Neon Postgres**
   - Go to: https://neon.tech
   - Sign up/Login
   - Create a new project
   - Copy: Connection string (starts with `postgresql://`)

### ✅ Step 2: Create Hugging Face Space (2 minutes)

1. Go to: https://huggingface.co/new-space
2. Fill in:
   - **Space name**: `rag-chatbot-backend` (or your choice)
   - **License**: Apache 2.0
   - **Select SDK**: **Docker**
   - **Space hardware**: CPU basic (free)
   - **Visibility**: Public
3. Click **Create Space**

### ✅ Step 3: Add Environment Secrets (3 minutes)

In your new Space:

1. Go to **Settings** tab
2. Scroll to **Repository secrets**
3. Click **New secret** and add each of these:

| Secret Name | Value |
|-------------|-------|
| `COHERE_API_KEY` | Your Cohere API key |
| `QDRANT_URL` | Your Qdrant cluster URL |
| `QDRANT_API_KEY` | Your Qdrant API key |
| `NEON_DB_URL` | Your Neon connection string |
| `DATABASE_URL` | Same as NEON_DB_URL |
| `DEBUG` | `false` |
| `ALLOWED_ORIGINS` | Your GitHub Pages URL |

Example for `ALLOWED_ORIGINS`:
```
https://your-username.github.io
```

### ✅ Step 4: Push Code (3 minutes)

```bash
# Navigate to backend folder
cd backend

# Add Hugging Face remote
git remote add hf https://huggingface.co/spaces/YOUR_USERNAME/rag-chatbot-backend

# Push to Hugging Face
git add .
git commit -m "Deploy backend to HF Spaces"
git push hf main
```

**Replace `YOUR_USERNAME` with your actual Hugging Face username**

### ✅ Step 5: Wait for Build (5-10 minutes)

1. Go to your Space page
2. Click **Logs** tab
3. Wait for "Application startup complete" message
4. Your API URL will be:
   ```
   https://YOUR_USERNAME-rag-chatbot-backend.hf.space
   ```

### ✅ Step 6: Test Your Backend

Open in browser:
```
https://YOUR_USERNAME-rag-chatbot-backend.hf.space/health
```

Should return:
```json
{"status": "healthy", "service": "RAG Chatbot API"}
```

### ✅ Step 7: View API Documentation

Visit:
```
https://YOUR_USERNAME-rag-chatbot-backend.hf.space/docs
```

You'll see interactive API documentation (Swagger UI).

## Next Steps

1. **Index Your Content**
   ```bash
   # Run locally to populate Qdrant with your book content
   cd backend
   python scripts/index_content.py
   ```

2. **Update Frontend**
   - Edit your Docusaurus chatbot component
   - Update API URL to your HF Space URL
   - Deploy frontend updates

3. **Test End-to-End**
   - Open your GitHub Pages site
   - Try the chatbot
   - Ask questions about your book content

## Troubleshooting

### Build Fails
- Check **Logs** tab for errors
- Verify all secrets are added correctly
- Ensure no syntax errors in code

### "Application startup failed"
- Check if DATABASE_URL is correct
- Verify Cohere API key is valid
- Check Qdrant connection

### 502 Bad Gateway
- Space might be starting up (wait 1-2 minutes)
- Check logs for application errors

### CORS Errors in Frontend
- Verify `ALLOWED_ORIGINS` includes your GitHub Pages URL
- Should be: `https://your-username.github.io`
- No trailing slash

## Need Help?

- HF Spaces Docs: https://huggingface.co/docs/hub/spaces
- Check your Space logs for detailed errors
- Ensure all environment variables are set correctly

## Alternative: Deploy with Hugging Face CLI

```bash
# Install CLI
pip install huggingface_hub

# Login
huggingface-cli login

# Create space
huggingface-cli repo create rag-chatbot-backend --type space --space_sdk docker

# Push code
cd backend
git remote add hf https://huggingface.co/spaces/YOUR_USERNAME/rag-chatbot-backend
git push hf main
```
