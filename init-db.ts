/**
 * Better Auth Database Initialization
 * Creates all required tables for Better Auth with student profile fields
 */

import Database from 'better-sqlite3';
import path from 'path';

const dbPath = process.env.DATABASE_URL?.replace('file:', '') || './auth.db';
const db = new Database(dbPath);

console.log(`📊 Initializing Better Auth database at: ${path.resolve(dbPath)}`);

// Enable foreign keys
db.pragma('foreign_keys = ON');

// Create user table with student profile fields
db.exec(`
  CREATE TABLE IF NOT EXISTS user (
    id TEXT PRIMARY KEY,
    email TEXT NOT NULL UNIQUE,
    emailVerified INTEGER NOT NULL DEFAULT 0,
    name TEXT NOT NULL,
    createdAt INTEGER NOT NULL,
    updatedAt INTEGER NOT NULL,
    image TEXT,

    -- Student Background
    softwareExperience TEXT DEFAULT 'beginner',
    hardwareExperience TEXT DEFAULT 'none',

    -- Course Progress
    enrollmentDate INTEGER,
    completedModules TEXT DEFAULT '[]',
    completedLabs TEXT DEFAULT '[]',
    currentModule INTEGER DEFAULT 1,
    progress INTEGER DEFAULT 0,

    -- Personalization
    preferredLanguage TEXT DEFAULT 'en',
    usePersonalization INTEGER DEFAULT 1,

    -- Profile
    fullName TEXT DEFAULT '',
    institution TEXT DEFAULT '',
    role TEXT DEFAULT 'student'
  );
`);

// Create account table (for password storage)
db.exec(`
  CREATE TABLE IF NOT EXISTS account (
    id TEXT PRIMARY KEY,
    accountId TEXT NOT NULL,
    providerId TEXT NOT NULL,
    userId TEXT NOT NULL,
    accessToken TEXT,
    refreshToken TEXT,
    idToken TEXT,
    expiresAt INTEGER,
    password TEXT,
    createdAt INTEGER NOT NULL,
    updatedAt INTEGER NOT NULL,
    FOREIGN KEY (userId) REFERENCES user(id) ON DELETE CASCADE,
    UNIQUE(providerId, accountId)
  );
`);

// Create session table
db.exec(`
  CREATE TABLE IF NOT EXISTS session (
    id TEXT PRIMARY KEY,
    userId TEXT NOT NULL,
    expiresAt INTEGER NOT NULL,
    token TEXT NOT NULL UNIQUE,
    ipAddress TEXT,
    userAgent TEXT,
    createdAt INTEGER NOT NULL,
    updatedAt INTEGER NOT NULL,
    FOREIGN KEY (userId) REFERENCES user(id) ON DELETE CASCADE
  );
`);

// Create verification table (for email verification)
db.exec(`
  CREATE TABLE IF NOT EXISTS verification (
    id TEXT PRIMARY KEY,
    identifier TEXT NOT NULL,
    value TEXT NOT NULL,
    expiresAt INTEGER NOT NULL,
    createdAt INTEGER NOT NULL,
    updatedAt INTEGER NOT NULL
  );
`);

// Create indexes for better performance
db.exec(`
  CREATE INDEX IF NOT EXISTS idx_user_email ON user(email);
  CREATE INDEX IF NOT EXISTS idx_session_userId ON session(userId);
  CREATE INDEX IF NOT EXISTS idx_session_token ON session(token);
  CREATE INDEX IF NOT EXISTS idx_account_userId ON account(userId);
  CREATE INDEX IF NOT EXISTS idx_verification_identifier ON verification(identifier);
`);

console.log('✅ Database tables created successfully!');
console.log('\nTables created:');
console.log('  - user (with student profile fields)');
console.log('  - account (for password authentication)');
console.log('  - session (for user sessions)');
console.log('  - verification (for email verification)');
console.log('\n📝 Student profile fields added to user table:');
console.log('  - softwareExperience, hardwareExperience');
console.log('  - completedModules, completedLabs, currentModule, progress');
console.log('  - preferredLanguage, usePersonalization');
console.log('  - fullName, institution, role');

db.close();
console.log('\n🎉 Database initialization complete!');
