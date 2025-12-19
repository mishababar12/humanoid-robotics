import express from 'express';
import { betterAuth } from 'better-auth';
import { sqliteAdapter } from 'better-auth/adapter';
import { toNodeHandler } from 'better-auth/node';
import Database from 'better-sqlite3';
import dotenv from 'dotenv';

// Load environment variables from .env file
dotenv.config();

console.log('Environment variables:');
console.log('- DATABASE_URL:', process.env.DATABASE_URL);
console.log('- AUTH_SECRET exists:', !!process.env.AUTH_SECRET);

// Create the database instance
const db = new Database(process.env.DATABASE_URL || './db.sqlite');

// Create the SQLite adapter
const adapter = sqliteAdapter(db, {
  // Options for the adapter if needed
});

console.log('SQLite adapter created');

// Initialize Better-Auth with the adapter
const auth = betterAuth({
  database: {
    provider: "sqlite",
    url: process.env.DATABASE_URL || "./db.sqlite",
    adapter: adapter // Use the custom adapter
  },
  secret: process.env.AUTH_SECRET || "fallback-secret-change-me",
  emailAndPassword: {
    enabled: true,
    requireEmailVerification: false,
  },
  session: {
    expiresIn: 7 * 24 * 60 * 60, // 7 days
  },
  debug: true
});

console.log('Better-Auth instance created with adapter');

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