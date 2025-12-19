import express from 'express';
import { toNodeHandler } from 'better-auth/node';
import { betterAuth } from 'better-auth';
import dotenv from 'dotenv';

// Load environment variables from .env file
dotenv.config();

console.log('Environment variables:');
console.log('- DATABASE_URL:', process.env.DATABASE_URL);
console.log('- AUTH_SECRET exists:', !!process.env.AUTH_SECRET);

// Initialize Better-Auth with minimal configuration to test database connection
const auth = betterAuth({
  database: {
    provider: "sqlite",
    url: process.env.DATABASE_URL || "./db.sqlite",
  },
  secret: process.env.AUTH_SECRET || "fallback-secret-change-me",
  // Minimal setup to test database connection
});

console.log('Better-Auth initialized successfully');

// Create a custom server that combines Docusaurus with Better-Auth API
const app = express();

// Parse JSON bodies
app.use(express.json());

// Mount Better-Auth API routes
app.use('/api/auth', toNodeHandler({
  auth,
  prefix: '/api/auth'
}));

// Health check endpoint
app.get('/health', (req, res) => {
  res.json({ status: 'ok', service: 'auth-server' });
});

// For the root route, respond with a message
app.get('/', (req, res) => {
  res.json({ message: 'Auth server running on port 3001' });
});

const PORT = process.env.PORT || 3001;
const server = app.listen(PORT, () => {
  console.log(`Auth server running on port ${PORT}`);
  console.log(`Better-Auth endpoints available at http://localhost:${PORT}/api/auth`);
});

// Handle potential errors
server.on('error', (err) => {
  console.error('Server error:', err);
});