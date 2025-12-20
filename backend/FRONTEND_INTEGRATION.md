# Frontend Integration Guide

## Connecting Your Docusaurus Site to the Backend

Once your backend is deployed on Hugging Face Spaces, you need to update your frontend to connect to it.

## Step 1: Find Your Backend URL

Your backend will be available at:
```
https://YOUR_USERNAME-SPACE_NAME.hf.space
```

Example:
```
https://johndoe-rag-chatbot-backend.hf.space
```

## Step 2: Create API Client

Create a file in your Docusaurus project:

**`src/utils/api.js`**

```javascript
// API Configuration
const API_BASE_URL = process.env.REACT_APP_API_URL ||
  'https://YOUR_USERNAME-SPACE_NAME.hf.space/api';

class ChatAPI {
  constructor() {
    this.baseURL = API_BASE_URL;
    this.sessionId = this.getOrCreateSessionId();
  }

  getOrCreateSessionId() {
    let sessionId = localStorage.getItem('chatSessionId');
    if (!sessionId) {
      sessionId = `session-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;
      localStorage.setItem('chatSessionId', sessionId);
    }
    return sessionId;
  }

  async chat(message, selectedText = null) {
    try {
      const response = await fetch(`${this.baseURL}/chat`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          message: message,
          selected_text: selectedText,
          session_id: this.sessionId,
          context: selectedText ? 'selected_text' : 'general'
        }),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();
      return data;
    } catch (error) {
      console.error('Chat API error:', error);
      throw error;
    }
  }

  async search(query) {
    try {
      const response = await fetch(`${this.baseURL}/search`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          query: query,
        }),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();
      return data;
    } catch (error) {
      console.error('Search API error:', error);
      throw error;
    }
  }

  async healthCheck() {
    try {
      const response = await fetch(`${this.baseURL}/health`);
      return response.ok;
    } catch (error) {
      console.error('Health check failed:', error);
      return false;
    }
  }

  clearSession() {
    localStorage.removeItem('chatSessionId');
    this.sessionId = this.getOrCreateSessionId();
  }
}

export default new ChatAPI();
```

## Step 3: Create Chat Component

**`src/components/ChatBot.jsx`**

```javascript
import React, { useState, useEffect, useRef } from 'react';
import ChatAPI from '../utils/api';
import './ChatBot.css';

export default function ChatBot() {
  const [messages, setMessages] = useState([]);
  const [input, setInput] = useState('');
  const [loading, setLoading] = useState(false);
  const [selectedText, setSelectedText] = useState(null);
  const [isOnline, setIsOnline] = useState(false);
  const messagesEndRef = useRef(null);

  // Check backend health on mount
  useEffect(() => {
    checkBackendHealth();
  }, []);

  const checkBackendHealth = async () => {
    const healthy = await ChatAPI.healthCheck();
    setIsOnline(healthy);
  };

  // Handle text selection in the page
  useEffect(() => {
    const handleSelection = () => {
      const selection = window.getSelection();
      const text = selection.toString().trim();
      if (text) {
        setSelectedText(text);
      }
    };

    document.addEventListener('mouseup', handleSelection);
    return () => document.removeEventListener('mouseup', handleSelection);
  }, []);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const handleSubmit = async (e) => {
    e.preventDefault();
    if (!input.trim() || loading) return;

    const userMessage = {
      role: 'user',
      content: input,
      selectedText: selectedText,
    };

    setMessages(prev => [...prev, userMessage]);
    setInput('');
    setLoading(true);

    try {
      const response = await ChatAPI.chat(input, selectedText);

      const botMessage = {
        role: 'assistant',
        content: response.answer,
        sources: response.sources,
      };

      setMessages(prev => [...prev, botMessage]);
      setSelectedText(null); // Clear selection after use
    } catch (error) {
      const errorMessage = {
        role: 'assistant',
        content: 'Sorry, I encountered an error. Please try again.',
        error: true,
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setLoading(false);
    }
  };

  const handleClearChat = () => {
    setMessages([]);
    ChatAPI.clearSession();
  };

  return (
    <div className="chatbot-container">
      <div className="chatbot-header">
        <h3>AI Assistant</h3>
        <div className="status-indicator">
          <span className={`status-dot ${isOnline ? 'online' : 'offline'}`}></span>
          <span>{isOnline ? 'Online' : 'Offline'}</span>
        </div>
        <button onClick={handleClearChat} className="clear-button">
          Clear Chat
        </button>
      </div>

      {selectedText && (
        <div className="selected-text-banner">
          <span>📝 Selected text: "{selectedText.substring(0, 50)}..."</span>
          <button onClick={() => setSelectedText(null)}>✕</button>
        </div>
      )}

      <div className="messages-container">
        {messages.length === 0 && (
          <div className="welcome-message">
            <p>👋 Hello! I'm your AI assistant for this book.</p>
            <p>Ask me anything or select text on the page for context-aware help!</p>
          </div>
        )}

        {messages.map((msg, idx) => (
          <div key={idx} className={`message ${msg.role}`}>
            <div className="message-content">
              {msg.content}
              {msg.sources && msg.sources.length > 0 && (
                <div className="sources">
                  <strong>Sources:</strong>
                  <ul>
                    {msg.sources.map((source, i) => (
                      <li key={i}>
                        <a href={source.url}>{source.title}</a>
                      </li>
                    ))}
                  </ul>
                </div>
              )}
            </div>
          </div>
        ))}

        {loading && (
          <div className="message assistant loading">
            <div className="typing-indicator">
              <span></span>
              <span></span>
              <span></span>
            </div>
          </div>
        )}

        <div ref={messagesEndRef} />
      </div>

      <form onSubmit={handleSubmit} className="input-form">
        <input
          type="text"
          value={input}
          onChange={(e) => setInput(e.target.value)}
          placeholder="Ask a question..."
          disabled={loading || !isOnline}
        />
        <button type="submit" disabled={loading || !isOnline || !input.trim()}>
          Send
        </button>
      </form>
    </div>
  );
}
```

## Step 4: Add Styles

**`src/components/ChatBot.css`**

```css
.chatbot-container {
  display: flex;
  flex-direction: column;
  height: 600px;
  max-width: 800px;
  margin: 2rem auto;
  border: 1px solid #e0e0e0;
  border-radius: 12px;
  overflow: hidden;
  box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
}

.chatbot-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 1rem;
  background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
  color: white;
}

.status-indicator {
  display: flex;
  align-items: center;
  gap: 0.5rem;
  font-size: 0.875rem;
}

.status-dot {
  width: 8px;
  height: 8px;
  border-radius: 50%;
  background-color: #ef4444;
}

.status-dot.online {
  background-color: #10b981;
}

.selected-text-banner {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 0.75rem;
  background: #fef3c7;
  border-bottom: 1px solid #fbbf24;
  font-size: 0.875rem;
}

.messages-container {
  flex: 1;
  overflow-y: auto;
  padding: 1rem;
  background: #f9fafb;
}

.welcome-message {
  text-align: center;
  padding: 2rem;
  color: #6b7280;
}

.message {
  margin-bottom: 1rem;
  padding: 0.75rem 1rem;
  border-radius: 8px;
  max-width: 80%;
}

.message.user {
  background: #667eea;
  color: white;
  margin-left: auto;
}

.message.assistant {
  background: white;
  color: #1f2937;
  border: 1px solid #e5e7eb;
}

.typing-indicator {
  display: flex;
  gap: 4px;
}

.typing-indicator span {
  width: 8px;
  height: 8px;
  border-radius: 50%;
  background: #9ca3af;
  animation: typing 1.4s infinite;
}

.typing-indicator span:nth-child(2) {
  animation-delay: 0.2s;
}

.typing-indicator span:nth-child(3) {
  animation-delay: 0.4s;
}

@keyframes typing {
  0%, 60%, 100% {
    transform: translateY(0);
  }
  30% {
    transform: translateY(-10px);
  }
}

.sources {
  margin-top: 0.5rem;
  padding-top: 0.5rem;
  border-top: 1px solid #e5e7eb;
  font-size: 0.875rem;
}

.input-form {
  display: flex;
  gap: 0.5rem;
  padding: 1rem;
  background: white;
  border-top: 1px solid #e5e7eb;
}

.input-form input {
  flex: 1;
  padding: 0.75rem;
  border: 1px solid #d1d5db;
  border-radius: 8px;
  font-size: 1rem;
}

.input-form button {
  padding: 0.75rem 1.5rem;
  background: #667eea;
  color: white;
  border: none;
  border-radius: 8px;
  cursor: pointer;
  font-weight: 500;
}

.input-form button:disabled {
  background: #d1d5db;
  cursor: not-allowed;
}
```

## Step 5: Add to Your Docusaurus Page

**`docs/intro.md`** (or any page)

```mdx
---
sidebar_position: 1
---

import ChatBot from '@site/src/components/ChatBot';

# Welcome

Your content here...

## Ask the AI Assistant

<ChatBot />
```

## Step 6: Configure Environment Variables

Create **`.env`** in your Docusaurus root:

```bash
REACT_APP_API_URL=https://YOUR_USERNAME-SPACE_NAME.hf.space/api
```

## Step 7: Update Docusaurus Config

**`docusaurus.config.js`**

```javascript
module.exports = {
  // ... other config
  customFields: {
    apiUrl: process.env.REACT_APP_API_URL || 'https://YOUR_USERNAME-SPACE_NAME.hf.space/api',
  },
};
```

## Testing Locally

1. Start your backend locally:
   ```bash
   cd backend
   python -m app.main
   ```

2. Update `.env` for local testing:
   ```bash
   REACT_APP_API_URL=http://localhost:8000/api
   ```

3. Start Docusaurus:
   ```bash
   cd my-website
   npm start
   ```

## Deployment Checklist

- [ ] Backend deployed to Hugging Face Spaces
- [ ] Environment secrets added to HF Space
- [ ] Content indexed in Qdrant
- [ ] Frontend updated with correct API URL
- [ ] CORS configured with GitHub Pages URL
- [ ] Local testing completed
- [ ] Frontend deployed to GitHub Pages
- [ ] End-to-end testing completed

## Troubleshooting

### CORS Errors
Update backend `ALLOWED_ORIGINS` environment variable:
```
ALLOWED_ORIGINS=https://your-username.github.io,http://localhost:3000
```

### Backend Not Responding
- Check HF Space is running
- Verify API URL is correct
- Check browser console for errors

### No Search Results
- Ensure content indexing completed
- Check Qdrant has data
- Verify Cohere API key is valid
