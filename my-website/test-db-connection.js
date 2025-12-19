import Database from 'better-sqlite3';
import dotenv from 'dotenv';

dotenv.config();

console.log('Testing database connection...');
console.log('DATABASE_URL from env:', process.env.DATABASE_URL);

const dbPath = process.env.DATABASE_URL || './db.sqlite';
console.log('Using database path:', dbPath);

try {
  const db = new Database(dbPath);
  console.log('Database connection successful');

  // Test basic operations
  const result = db.prepare('SELECT sqlite_version() as version').get();
  console.log('SQLite version:', result.version);

  // Create a simple table to test write operations
  db.exec('CREATE TABLE IF NOT EXISTS test_table (id INTEGER PRIMARY KEY, name TEXT)');
  console.log('Test table created successfully');

  // Insert a test record
  const insert = db.prepare('INSERT INTO test_table (name) VALUES (?)');
  const info = insert.run('test_record');
  console.log('Test record inserted with id:', info.lastInsertRowid);

  // Read the test record
  const select = db.prepare('SELECT * FROM test_table WHERE id = ?').get(info.lastInsertRowid);
  console.log('Retrieved test record:', select);

  db.close();
  console.log('Database test completed successfully');
} catch (error) {
  console.error('Database test failed:', error.message);
}