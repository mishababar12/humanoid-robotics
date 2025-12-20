# Backend Deployment Guide - Hugging Face Spaces

## Prerequisites
1. Hugging Face account (https://huggingface.co)
2. Git configured with your credentials
3. API keys ready:
   - Cohere API Key
   - Qdrant URL & API Key
   - Neon Postgres DB URL

## Step 1: Create Hugging Face Space

1. **Create New Space**
   - Go to https://huggingface.co/new-space
   - Choose a name (e.g., `your-username/rag-chatbot-backend`)
   - Select: **Docker** as Space SDK
   - Select: **Public** or **Private** based on your preference
   - Click **Create Space**

## Step 2: Configure Environment Variables

In your Hugging Face Space settings:

1. Go to **Settings** tab
2. Scroll to **Repository secrets**
3. Add these secrets:

```
COHERE_API_KEY=your_actual_cohere_key
QDRANT_URL=your_actual_qdrant_url
QDRANT_API_KEY=your_actual_qdrant_key
NEON_DB_URL=your_actual_neon_db_url
DATABASE_URL=your_actual_neon_db_url
DEBUG=false
```

## Step 3: Push Code to Hugging Face

### Option A: Using Git (Recommended)

```bash
# Navigate to backend directory
cd backend

# Initialize git if not already done
git init

# Add Hugging Face remote
git remote add hf https://huggingface.co/spaces/YOUR_USERNAME/YOUR_SPACE_NAME

# Add all files
git add .

# Commit
git commit -m "Initial backend deployment"

# Push to Hugging Face
git push hf main
```

### Option B: Using Hugging Face CLI

```bash
# Install Hugging Face CLI
pip install huggingface_hub

# Login
huggingface-cli login

# Upload files
huggingface-cli upload-large-folder . YOUR_USERNAME/YOUR_SPACE_NAME
```

## Step 4: Verify Deployment

1. Wait for build to complete (check **Logs** tab in your Space)
2. Once running, your API will be available at:
   ```
   https://YOUR_USERNAME-YOUR_SPACE_NAME.hf.space
   ```
3. Test the health endpoint:
   ```
   https://YOUR_USERNAME-YOUR_SPACE_NAME.hf.space/health
   ```

## Step 5: Index Your Content

Before the chatbot can work, you need to index your Docusaurus content:

```bash
# Run the indexing script (do this locally or via Space terminal)
python scripts/index_content.py
```

Or create a startup script that automatically indexes content on deployment.

## Step 6: Update Frontend

Update your Docusaurus frontend to use the deployed backend URL:

```javascript
// In your frontend chat component
const API_URL = 'https://YOUR_USERNAME-YOUR_SPACE_NAME.hf.space/api';
```

## Troubleshooting

### Build Fails
- Check **Logs** tab for error messages
- Verify Dockerfile syntax
- Ensure all requirements are in requirements.txt

### Environment Variables Not Working
- Verify secrets are added in Space settings
- Restart the Space after adding secrets
- Check logs for "missing environment variable" errors

### Database Connection Issues
- Verify Neon DB URL is correct
- Check if Neon DB allows connections from external IPs
- Ensure DATABASE_URL matches NEON_DB_URL format

### Port Issues
- Hugging Face Spaces use port 7860 by default
- Your Dockerfile already uses port 7860 (correctly configured)

## Monitoring

- View logs: Space **Logs** tab
- Check metrics: Space **Analytics** tab
- API docs: `https://YOUR_USERNAME-YOUR_SPACE_NAME.hf.space/docs`

## Updating the Deployment

```bash
# Make changes to code
# Commit and push
git add .
git commit -m "Update: description of changes"
git push hf main

# Space will automatically rebuild and redeploy
```

## Free Tier Limitations

Hugging Face Spaces Free Tier:
- 16 GB RAM
- 8 CPU cores
- 50 GB storage
- May sleep after inactivity (wakes on request)

For production or heavy usage, consider upgrading to a paid tier.

## Next Steps

1. Set up automatic content indexing on deployment
2. Configure CORS properly for your frontend domain
3. Set up monitoring and error tracking
4. Consider adding authentication for production use
5. Optimize Docker image size for faster builds

## Support

- Hugging Face Docs: https://huggingface.co/docs/hub/spaces
- Issues: Create an issue in this repository
