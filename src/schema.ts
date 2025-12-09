/**
 * Database schema for Better Auth and application data
 * Defines the tables for authentication and student profiles
 */

import { sqliteTable, text, integer, primaryKey } from 'drizzle-orm/sqlite-core';
import { sql } from 'drizzle-orm';

// User table with extended student profile fields
export const users = sqliteTable('user', {
  id: text('id').primaryKey(),
  email: text('email').notNull().unique(),
  emailVerified: integer('emailVerified', { mode: 'boolean' }).default(false),
  name: text('name').notNull(),
  createdAt: integer('createdAt', { mode: 'timestamp' }).default(sql`CURRENT_TIMESTAMP`),
  updatedAt: integer('updatedAt', { mode: 'timestamp' }).$onUpdate(() => new Date()),
  image: text('image'),
  
  // Student profile fields
  softwareExperience: text('softwareExperience').default('beginner'),
  hardwareExperience: text('hardwareExperience').default('none'),
  enrollmentDate: integer('enrollmentDate', { mode: 'timestamp' }),
  completedModules: text('completedModules').default('[]'), // JSON string
  completedLabs: text('completedLabs').default('[]'), // JSON string
  currentModule: integer('currentModule').default(1),
  progress: integer('progress').default(0),
  preferredLanguage: text('preferredLanguage').default('en'),
  usePersonalization: integer('usePersonalization', { mode: 'boolean' }).default(true),
  fullName: text('fullName').default(''),
  institution: text('institution').default(''),
  role: text('role').default('student'),
});

// Account table for authentication providers
export const accounts = sqliteTable('account', {
  id: text('id').primaryKey(),
  accountId: text('accountId').notNull(),
  providerId: text('providerId').notNull(),
  userId: text('userId')
    .notNull()
    .references(() => users.id, { onDelete: 'cascade' }),
  accessToken: text('accessToken'),
  refreshToken: text('refreshToken'),
  idToken: text('idToken'),
  expiresAt: integer('expiresAt', { mode: 'timestamp' }),
  password: text('password'),
  createdAt: integer('createdAt', { mode: 'timestamp' }).default(sql`CURRENT_TIMESTAMP`),
  updatedAt: integer('updatedAt', { mode: 'timestamp' }).$onUpdate(() => new Date()),
});

// Session table
export const sessions = sqliteTable('session', {
  id: text('id').primaryKey(),
  userId: text('userId')
    .notNull()
    .references(() => users.id, { onDelete: 'cascade' }),
  expiresAt: integer('expiresAt', { mode: 'timestamp' }).notNull(),
  token: text('token').notNull().unique(),
  ipAddress: text('ipAddress'),
  userAgent: text('userAgent'),
  createdAt: integer('createdAt', { mode: 'timestamp' }).default(sql`CURRENT_TIMESTAMP`),
  updatedAt: integer('updatedAt', { mode: 'timestamp' }).$onUpdate(() => new Date()),
});

// Verification table for email verification
export const verifications = sqliteTable('verification', {
  id: text('id').primaryKey(),
  identifier: text('identifier').notNull(),
  value: text('value').notNull(),
  expiresAt: integer('expiresAt', { mode: 'timestamp' }).notNull(),
  createdAt: integer('createdAt', { mode: 'timestamp' }).default(sql`CURRENT_TIMESTAMP`),
  updatedAt: integer('updatedAt', { mode: 'timestamp' }).$onUpdate(() => new Date()),
});

// Export all schema for Drizzle
export const schema = {
  users,
  accounts,
  sessions,
  verifications,
};