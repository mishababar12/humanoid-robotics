import Database from 'better-sqlite3';
import { existsSync } from 'fs';

const dbPath = './db.sqlite';
const db = new Database(dbPath);

// Create the necessary tables for Better-Auth
// This is a simplified version - Better-Auth will handle the actual schema
console.log('Initializing database at:', dbPath);

// Test the database connection
try {
  const result = db.prepare('SELECT 1 as test').get();
  console.log('Database connection successful:', result);

  // Create a simple test table to ensure the database is working
  db.exec(`
    CREATE TABLE IF NOT EXISTS _better_auth_test (
      id INTEGER PRIMARY KEY,
      created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
    )
  `);

  console.log('Database initialization completed successfully');
  db.close();
} catch (error) {
  console.error('Database initialization failed:', error);
  process.exit(1);
}