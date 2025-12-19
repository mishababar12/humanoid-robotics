import { betterAuth } from 'better-auth';
import dotenv from 'dotenv';

// Load environment variables from .env file
dotenv.config();

console.log('Environment variables:');
console.log('- DATABASE_URL:', process.env.DATABASE_URL);
console.log('- AUTH_SECRET exists:', !!process.env.AUTH_SECRET);

try {
  // Initialize Better-Auth with basic configuration
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
    session: {
      expiresIn: 7 * 24 * 60 * 60, // 7 days
    }
  });

  console.log('Better-Auth initialized successfully!');
  console.log('Auth object keys:', Object.keys(auth));
} catch (error) {
  console.error('Better-Auth initialization failed:', error.message);
  console.error('Error cause:', error.cause);
  console.error('Full error:', error);
}