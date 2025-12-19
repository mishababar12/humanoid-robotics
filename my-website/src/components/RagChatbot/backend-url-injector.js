// This file sets the backend URL as a global variable for the RagChatbot component
// It's imported in the Docusaurus config

// Set the backend URL as a global variable
if (typeof window !== 'undefined') {
  // In browser environment, we'll use the backend URL from Docusaurus config
  // The backend URL is passed through Docusaurus customFields
  const backendUrl = window.location.hostname === 'localhost'
    ? 'http://localhost:8000'  // Use port 8000 for local development
    : 'https://site.vercel.app'; // Use production URL for production

  window.BACKEND_URL = backendUrl;
  console.log('Backend URL injected:', backendUrl);
}