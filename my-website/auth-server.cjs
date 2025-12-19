// auth-server.cjs - CommonJS format
const express = require('express');
const { toNodeHandler } = require('better-auth/node');
const { betterAuth } = require('better-auth');
const dotenv = require('dotenv');

// Load environment variables from .env file
dotenv.config();

// Initialize Better-Auth with proper database configuration
const auth = betterAuth({
  database: {
    provider: "sqlite",
    url: process.env.DATABASE_URL || "./db.sqlite",
  },
  secret: process.env.AUTH_SECRET || "fallback-secret-change-me",
  emailAndPassword: {
    enabled: true,
    requireEmailVerification: false,
  },
  socialProviders: {
    // Add social providers if needed
  },
  user: {
    // Additional user fields for background questionnaire
    additionalFields: {
      expertiseLevel: {
        type: "string",
        required: false,
      },
      background: {
        type: "string",
        required: false,
      },
      softwareExperience: {
        type: "string",
        required: false,
      },
      hardwareExperience: {
        type: "string",
        required: false,
      },
    },
  },
  session: {
    expiresIn: 7 * 24 * 60 * 60, // 7 days
  }
});

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
