import express from 'express';
import { toNodeHandler } from 'better-auth/node';
import { auth } from './src/auth/auth.config.js';
import * as path from 'path';
import { fileURLToPath } from 'url';

// Get the current directory name for file operations
const __filename = fileURLToPath(import.meta.url);
const __dirname = path.dirname(__filename);

// Create a custom server that combines Docusaurus with Better-Auth API
const app = express();

// Parse JSON bodies
app.use(express.json());

// Mount Better-Auth API routes
app.use('/api/auth', toNodeHandler({
  auth,
  prefix: '/api/auth'
}));

// Serve static files from Docusaurus build
app.use(express.static(path.join(__dirname, 'build')));

// For other routes, serve the Docusaurus index.html
app.get(/.*/, (req, res) => {
  res.sendFile(path.join(__dirname, 'build', 'index.html'));
});

// Health check endpoint
app.get('/health', (req, res) => {
  res.json({ status: 'ok', service: 'auth-server' });
});

const PORT = process.env.PORT || 3001;
const server = app.listen(PORT, () => {
  console.log(`Auth server running on port ${PORT}`);
  console.log(`Better-Auth endpoints available at /api/auth`);
});

// Handle uncaught exceptions and unhandled rejections to prevent crashes
process.on('uncaughtException', (err) => {
  console.error('Uncaught Exception:', err);
});

process.on('unhandledRejection', (reason, promise) => {
  console.error('Unhandled Rejection at:', promise, 'reason:', reason);
});