/**
 * Database setup for Better Auth
 * Creates and exports the Drizzle database instance
 */

import Database from 'better-sqlite3';
import { drizzle } from 'drizzle-orm/better-sqlite3';
import * as schema from './schema'; // Import your schema file

// Connect to SQLite database
const sqlite = new Database(process.env.DATABASE_URL?.replace('file:', '') || './db.sqlite');

// Create Drizzle ORM instance
export const db = drizzle(sqlite, { schema });

// Export the database connection too if needed
export { sqlite };