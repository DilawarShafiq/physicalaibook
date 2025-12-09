/**
 * Better Auth Configuration
 * Handles user authentication with student profile data
 */

import { betterAuth } from "better-auth";
import Database from "better-sqlite3";

const sqlite = new Database(process.env.DATABASE_URL?.replace('file:', '') || "./auth.db");

export const auth = betterAuth({
  database: {
    provider: "sqlite",
    db: sqlite,
  },
  emailAndPassword: {
    enabled: true,
    requireEmailVerification: false, // Set to true in production
    minPasswordLength: 8,
    maxPasswordLength: 128,
  },
  user: {
    additionalFields: {
      // Student Background
      softwareExperience: {
        type: "string",
        required: false,
        defaultValue: "beginner",
        input: true,
      },
      hardwareExperience: {
        type: "string",
        required: false,
        defaultValue: "none",
        input: true,
      },
      // Course Progress
      enrollmentDate: {
        type: "date",
        required: false,
        defaultValue: () => new Date(),
      },
      completedModules: {
        type: "string", // JSON array of completed module IDs
        required: false,
        defaultValue: "[]",
      },
      completedLabs: {
        type: "string", // JSON array of completed lab IDs
        required: false,
        defaultValue: "[]",
      },
      currentModule: {
        type: "number",
        required: false,
        defaultValue: 1,
      },
      progress: {
        type: "number", // Percentage 0-100
        required: false,
        defaultValue: 0,
      },
      // Personalization Preferences
      preferredLanguage: {
        type: "string",
        required: false,
        defaultValue: "en",
      },
      usePersonalization: {
        type: "boolean",
        required: false,
        defaultValue: true,
      },
      // Profile
      fullName: {
        type: "string",
        required: false,
        defaultValue: "",
        input: true,
      },
      institution: {
        type: "string",
        required: false,
        defaultValue: "",
      },
      role: {
        type: "string",
        required: false,
        defaultValue: "student", // student, instructor, admin
      },
    },
  },
  session: {
    expiresIn: 60 * 60 * 24 * 7, // 7 days
    updateAge: 60 * 60 * 24, // Update session every 1 day
    cookieCache: {
      enabled: true,
      maxAge: 60 * 5, // Cache for 5 minutes
    },
  },
  trustedOrigins: [
    "http://localhost:3000",
    "http://localhost:3001",
    "http://127.0.0.1:3000",
    process.env.BETTER_AUTH_URL || "",
  ].filter(Boolean),
  advanced: {
    generateId: () => crypto.randomUUID(),
  },
});

export type Session = typeof auth.$Infer.Session.session;
export type User = typeof auth.$Infer.Session.user;
