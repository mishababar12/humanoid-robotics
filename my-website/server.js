import express from 'express';
import { toNodeHandler } from 'better-auth/node';
import { auth } from './src/auth/auth.config.js';

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