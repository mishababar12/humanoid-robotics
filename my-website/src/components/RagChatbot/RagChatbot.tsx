import React, { useState, useRef, useEffect } from 'react';
// import { useUserPersonalization } from '../../contexts/UserPersonalizationContext';
import './RagChatbot.css';

interface Message {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  sources?: Array<{
    title: string;
    url: string;
    content_preview: string;
  }>;
  timestamp: string;
}

const RagChatbot: React.FC = () => {
  const [messages, setMessages] = useState<Message[]>([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [selectedText, setSelectedText] = useState<string | null>(null);
  const [sessionID, setSessionID] = useState<string>('');
  const [isExpanded, setIsExpanded] = useState(false);
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const textareaRef = useRef<HTMLTextAreaElement>(null);

  // Personalization context - disabled for now
  // const { getPersonalizedPrompt } = useUserPersonalization();
  const getPersonalizedPrompt = (prompt: string) => prompt; // Simple passthrough

  // Auto-scroll to bottom when messages change
  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  // Handle text selection on the page
  useEffect(() => {
    const handleSelection = () => {
      const selectedText = window.getSelection()?.toString().trim();
      if (selectedText && selectedText.length > 10) { // Only capture meaningful selections
        setSelectedText(selectedText);
      }
    };

    document.addEventListener('mouseup', handleSelection);
    return () => {
      document.removeEventListener('mouseup', handleSelection);
    };
  }, []);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    if (!inputValue.trim() || isLoading) return;

    // Add user message
    const userMessage: Message = {
      id: Date.now().toString(),
      role: 'user',
      content: inputValue,
      timestamp: new Date().toISOString(),
    };

    setMessages(prev => [...prev, userMessage]);
    const currentInput = inputValue.trim().toLowerCase();
    setInputValue('');
    setIsLoading(true);

    // Check for greetings
    const greetings = ['hi', 'hello', 'hey', 'greetings', 'good morning', 'good afternoon', 'good evening', 'salam', 'assalam', 'assalamualaikum'];
    const isGreeting = greetings.some(greeting =>
      currentInput === greeting ||
      currentInput.startsWith(greeting + ' ') ||
      currentInput.startsWith(greeting + ',')
    );

    if (isGreeting) {
      // Respond with greeting without calling backend
      setTimeout(() => {
        const greetingResponse: Message = {
          id: Date.now().toString(),
          role: 'assistant',
          content: 'Hello! How can I help you today? Feel free to ask me anything about the content on this page.',
          timestamp: new Date().toISOString(),
        };
        setMessages(prev => [...prev, greetingResponse]);
        setIsLoading(false);
      }, 500);
      return;
    }

    try {
      // Create session if not exists
      if (!sessionID) {
        // In Docusaurus, we use a global variable that will be set by the site
        const apiBaseUrl = String(window.BACKEND_URL || 'http://localhost:8000');
        console.log('Creating new session with API base URL:', String(apiBaseUrl));
        const sessionResponse = await fetch(`${apiBaseUrl}/api/session`, {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({}),
        });

        console.log('Session response status:', String(sessionResponse.status));

        if (sessionResponse.ok) {
          const sessionData = await sessionResponse.json();
          console.log('Session created with ID:', String(sessionData.session_id));
          setSessionID(sessionData.session_id);
        } else {
          console.error('Failed to create session:', sessionResponse.status);
          // Don't throw here, just proceed without a session ID
          // The backend can handle requests without a session ID
        }
      }

      // Prepare the request body with personalized prompt
      const personalizedMessage = getPersonalizedPrompt(inputValue);
      console.log('personalizedMessage in RagChatbot:', String(personalizedMessage));
      const requestBody = {
        message: personalizedMessage,
        selected_text: selectedText || undefined,
        session_id: sessionID || undefined,
        context: selectedText ? "selected_text" : "general"
      };

      // Send request to backend - using environment variable
      const apiBaseUrl = String(window.BACKEND_URL || 'http://localhost:8000');
      console.log('Sending request to:', String(`${apiBaseUrl}/api/chat`), 'with data:', JSON.stringify(requestBody));

      const response = await fetch(`${apiBaseUrl}/api/chat`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(requestBody),
      });

      console.log('Response status:', String(response.status));

      if (!response.ok) {
        // Try to get error details from response
        let errorText = `API error: ${response.status}`;
        try {
          const errorData = await response.json();
          console.error('API Error Response:', errorData);
          errorText += ` - ${errorData.detail || errorData.message || 'Unknown error'}`;
        } catch (e) {
          // If we can't parse the error, use the status text
          errorText = `API error: ${response.status} ${response.statusText}`;
          console.error('API Error (non-JSON):', errorText);
        }
        throw new Error(errorText);
      }

      const data = await response.json();
      console.log('API Response data:', JSON.stringify(data));

      // Check if the response contains an error message from the backend
      if (data.response && (data.response.includes("error") || data.response.includes("Error"))) {
        // This might be a backend-generated error response, but we'll still process it
        console.warn("Backend returned an error response:", data.response);
      }

      // Add assistant message
      const assistantMessage: Message = {
        id: Date.now().toString(),
        role: 'assistant',
        content: typeof data.response === 'string' ? data.response : JSON.stringify(data.response),
        sources: data.sources,
        timestamp: typeof data.timestamp === 'string' ? data.timestamp : new Date().toISOString(),
      };

      setMessages(prev => [...prev, assistantMessage]);

      // Update session ID if not set
      if (!sessionID && data.session_id) {
        setSessionID(data.session_id);
      }

      // Clear selected text after use
      setSelectedText(null);
    } catch (error) {
      console.error('Chat error:', error);
      let errorMessageContent = 'Sorry, I encountered an error. Please try again.';

      // Provide more specific error messages based on the error type
      if (error instanceof TypeError && error.message.includes('fetch')) {
        errorMessageContent = 'Unable to connect to the server. Please make sure the backend is running.';
      } else if (error instanceof Error) {
        errorMessageContent = `Error: ${error.message}`;
      }

      const errorMessage: Message = {
        id: Date.now().toString(),
        role: 'assistant',
        content: errorMessageContent,
        timestamp: new Date().toISOString(),
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyDown = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSubmit(e as any);
    }
  };

  const clearChat = () => {
    setMessages([]);
    setSessionID('');
    setSelectedText(null);
  };

  return (
    <div className={`rag-chatbot-container ${isExpanded ? 'expanded' : 'collapsed'}`}>
      <div className="chatbot-header">
        <div className="chatbot-title" onClick={() => setIsExpanded(!isExpanded)}>📚 AI Textbook Assistant</div>
        <div className="chatbot-controls">
          {messages.length > 0 && (
            <button
              onClick={(e) => { e.stopPropagation(); clearChat(); }}
              className="clear-btn"
              title="Clear chat"
            >
              ✕
            </button>
          )}
          {isExpanded ? (
            <button
              onClick={(e) => { e.stopPropagation(); setIsExpanded(false); }}
              className="close-btn"
              title="Close chatbot"
            >
              ×
            </button>
          ) : (
            <button
              onClick={(e) => { e.stopPropagation(); setIsExpanded(true); }}
              className="expand-btn"
              title="Expand"
            >
              +
            </button>
          )}
        </div>
      </div>

      {isExpanded && (
        <div className="chatbot-content">
          {selectedText && (
            <div className="selected-text-preview">
              <strong>Selected Text:</strong> "{selectedText.substring(0, 100)}{selectedText.length > 100 ? '...' : ''}"
            </div>
          )}

          <div className="messages-container">
            {messages.length === 0 ? (
              <div className="welcome-message">
                <p>Ask questions about the content on this page</p>
              </div>
            ) : (
              messages.map((message) => (
                <div
                  key={message.id}
                  className={`message ${message.role}`}
                >
                  <div className="message-content">
                    {message.content}
                  </div>
                  {message.sources && message.sources.length > 0 && (
                    <div className="sources">
                      <details>
                        <summary>Sources:</summary>
                        <ul>
                          {message.sources.slice(0, 3).map((source, index) => (
                            <li key={index}>
                              <a href={source.url} target="_blank" rel="noopener noreferrer">
                                {source.title}
                              </a>
                              <p className="source-preview">{source.content_preview}</p>
                            </li>
                          ))}
                        </ul>
                      </details>
                    </div>
                  )}
                </div>
              ))
            )}
            {isLoading && (
              <div className="message assistant">
                <div className="message-content">
                  <div className="typing-indicator">
                    <span></span>
                    <span></span>
                    <span></span>
                  </div>
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          <form className="input-form" onSubmit={handleSubmit}>
            <textarea
              ref={textareaRef}
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              onKeyDown={handleKeyDown}
              placeholder={selectedText
                ? "Ask about the selected text..."
                : "Ask a question about this textbook content..."}
              rows={2}
              disabled={isLoading}
              className="chat-input"
            />
            <button
              type="submit"
              disabled={!inputValue.trim() || isLoading}
              className="send-button"
            >
              →
            </button>
          </form>
        </div>
      )}
    </div>
  );
};

export default RagChatbot;