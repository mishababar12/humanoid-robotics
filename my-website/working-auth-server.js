import express from 'express';
import { toNodeHandler } from 'better-auth/node';
import { betterAuth } from 'better-auth';
import dotenv from 'dotenv';

// Load environment variables from .env file
dotenv.config();

console.log('Environment variables:');
console.log('- DATABASE_URL:', process.env.DATABASE_URL);
console.log('- AUTH_SECRET exists:', !!process.env.AUTH_SECRET);

// Initialize Better-Auth with configuration that should work with SQLite
const auth = betterAuth({
  database: {
    provider: "sqlite",
    url: process.env.DATABASE_URL || "./db.sqlite",
    // Add SQLite-specific options that might help
  },
  secret: process.env.AUTH_SECRET || "fallback-secret-change-me",
  emailAndPassword: {
    enabled: true,
    requireEmailVerification: false,
  },
  // Disable social providers initially
  account: {
    accountModel: {
      createOnNoAccount: true
    }
  },
  session: {
    expiresIn: 7 * 24 * 60 * 60, // 7 days
  },
  // Add debug info
  debug: true
});

console.log('Better-Auth instance created');

// Create a custom server that combines Docusaurus with Better-Auth API
const app = express();

// Parse JSON bodies
app.use(express.json());

// Mount Better-Auth API routes
const authHandler = toNodeHandler({
  auth,
  prefix: '/api/auth'
});

app.use('/api/auth', (req, res, next) => {
  console.log(`Auth endpoint hit: ${req.method} ${req.path}`);
  authHandler(req, res, next);
});

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