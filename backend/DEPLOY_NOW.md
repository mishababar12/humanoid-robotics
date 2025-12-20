# 🚀 Deploy RIGHT NOW - Step by Step

## Step 1: Create Hugging Face Account (2 minutes)

### Action Required:
1. **Open this link:** https://huggingface.co/join
2. Sign up with:
   - Email aur password
   - Ya GitHub account se login
3. Email verify karo
4. Login ho jao

✅ **Done? Next step par jao**

---

## Step 2: Create New Space (2 minutes)

### Action Required:
1. **Open this link:** https://huggingface.co/new-space

2. **Fill the form:**
   ```
   Owner: YOUR_USERNAME (automatically selected)
   Space name: rag-chatbot-backend
   License: Apache 2.0
   Select the Space SDK: Docker (IMPORTANT!)
   Space hardware: CPU basic - Free
   Space visibility: Public
   ```

3. **Click "Create Space"**

4. **IMPORTANT:** Space ka URL note karo:
   ```
   https://huggingface.co/spaces/YOUR_USERNAME/rag-chatbot-backend
   ```

✅ **Space created? Next step par jao**

---

## Step 3: Add Your API Keys as Secrets (3 minutes)

### Action Required:

Space create hone ke baad:

1. **Go to Settings tab** (top of your space page)

2. **Scroll down to "Repository secrets"**

3. **Click "New secret"** and add each one:

### Add These Secrets:

#### Secret 1:
```
Name: COHERE_API_KEY
Value: [YOUR ACTUAL COHERE KEY - paste here]
```

#### Secret 2:
```
Name: QDRANT_URL
Value: [YOUR QDRANT CLUSTER URL - paste here]
```

#### Secret 3:
```
Name: QDRANT_API_KEY
Value: [YOUR QDRANT API KEY - paste here]
```

#### Secret 4:
```
Name: NEON_DB_URL
Value: [YOUR NEON CONNECTION STRING - paste here]
```

#### Secret 5:
```
Name: DATABASE_URL
Value: [SAME AS NEON_DB_URL - paste here]
```

#### Secret 6:
```
Name: DEBUG
Value: false
```

#### Secret 7:
```
Name: ALLOWED_ORIGINS
Value: https://mishababar12.github.io
```

**⚠️ IMPORTANT:** Har secret ko "Add secret" button se save karo!

✅ **All secrets added? Next step par jao**

---

## Step 4: Push Code to Hugging Face (2 minutes)

### Action Required:

Open your terminal/command prompt:

```bash
# Backend folder mein jao
cd C:\Users\DELL\my-hackathone-project\backend

# Hugging Face remote add karo (replace YOUR_USERNAME)
git remote add hf https://huggingface.co/spaces/YOUR_USERNAME/rag-chatbot-backend

# Push karo
git add .
git commit -m "Deploy to HF Spaces"
git push hf main
```

**⚠️ Note:**
- `YOUR_USERNAME` ko apne actual HF username se replace karo
- Git credentials manga to HF username aur access token do
- Access token: https://huggingface.co/settings/tokens se banao

✅ **Push successful? Next step par jao**

---

## Step 5: Wait for Build (5-10 minutes)

### What's Happening:

HF Space ab Docker image build kar raha hai.

### How to Check:

1. Go to your Space page:
   ```
   https://huggingface.co/spaces/YOUR_USERNAME/rag-chatbot-backend
   ```

2. Click **"Logs"** tab (top)

3. Watch the build process

### Look For:
```
✓ Building Docker image...
✓ Starting application...
✓ Application startup complete
INFO: Uvicorn running on http://0.0.0.0:7860
```

**⚠️ If you see errors:**
- Check Logs carefully
- Verify all secrets are added correctly
- Check for typos in API keys

✅ **Build successful aur "running" show kar raha hai? Next step par jao**

---

## Step 6: Test Your Deployment (2 minutes)

### Test 1: Health Check

**Open in browser:**
```
https://YOUR_USERNAME-rag-chatbot-backend.hf.space/health
```

**Expected Response:**
```json
{
  "status": "healthy",
  "service": "RAG Chatbot API"
}
```

✅ **Working? Great!**

### Test 2: API Documentation

**Open in browser:**
```
https://YOUR_USERNAME-rag-chatbot-backend.hf.space/docs
```

**You should see:** Interactive API documentation (Swagger UI)

✅ **Docs loading? Perfect!**

### Test 3: Root Endpoint

**Open in browser:**
```
https://YOUR_USERNAME-rag-chatbot-backend.hf.space/
```

**Expected Response:**
```json
{
  "message": "RAG Chatbot API",
  "version": "1.0.0",
  "status": "running",
  "docs": "/docs for API documentation"
}
```

✅ **All tests passed? DEPLOYMENT SUCCESSFUL! 🎉**

---

## Step 7: Index Your Content (3 minutes)

### Action Required:

Locally run karo (backend folder mein):

```bash
cd C:\Users\DELL\my-hackathone-project\backend

# Update .env file with your deployed backend URL
# Then run:
python scripts/index_content.py
```

This will:
- Read your Docusaurus content
- Create embeddings
- Store in Qdrant

**Look for:**
```
✓ Indexed X documents
✓ Total chunks: XXX
✓ Upload complete
```

✅ **Indexing done? Awesome!**

---

## 🎯 YOUR BACKEND IS LIVE!

### Your Backend URL:
```
https://YOUR_USERNAME-rag-chatbot-backend.hf.space
```

### API Endpoints:
- Health: `/health`
- Chat: `/api/chat`
- Search: `/api/search`
- Docs: `/docs`

---

## Next: Connect Frontend

Follow **FRONTEND_INTEGRATION.md** to:
1. Update your Docusaurus site
2. Add ChatBot component
3. Configure API URL
4. Deploy frontend

---

## 🆘 Troubleshooting

### Build Failed?
- Check Logs tab
- Verify all secrets are correct
- Ensure no typos in secret names

### 502 Bad Gateway?
- Wait 2-3 minutes (cold start)
- Check Logs for errors
- Verify environment variables

### Can't Push to HF?
```bash
# Get access token from: https://huggingface.co/settings/tokens
# Use token as password when git asks for credentials
# Username: your HF username
# Password: your access token (starts with hf_...)
```

### CORS Errors?
- Verify ALLOWED_ORIGINS in secrets
- Should be: `https://mishababar12.github.io`

---

## ✅ Deployment Checklist

- [ ] HF account created
- [ ] Space created with Docker SDK
- [ ] All 7 secrets added
- [ ] Code pushed successfully
- [ ] Build completed (check Logs)
- [ ] Health endpoint working
- [ ] API docs loading
- [ ] Content indexed

**ALL CHECKED? DEPLOYMENT COMPLETE! 🚀🎉**
