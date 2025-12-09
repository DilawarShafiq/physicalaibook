/**
 * Better Auth Instance Configuration
 * Sets up the authentication service with student profile fields
 */

import { betterAuth } from "better-auth";
import { drizzleAdapter } from "better-auth/adapters/drizzle";
import { db } from "./database"; // Import your Drizzle DB instance

export const auth = betterAuth({
  database: drizzleAdapter(db, {
    provider: "sqlite", // or "postgresql" / "mysql" based on your setup
  }),
  secret: process.env.BETTER_AUTH_SECRET!,
  baseURL: process.env.BETTER_AUTH_URL,
  emailAndPassword: {
    enabled: true,
    requireEmailVerification: false,
    autoSignIn: true,
  },
  socialProviders: {
    // Add social providers as needed
  },
  user: {
    // Additional fields for student profiles
    additionalFields: {
      softwareExperience: {
        type: "string",
        defaultValue: "beginner",
      },
      hardwareExperience: {
        type: "string",
        defaultValue: "none",
      },
      enrollmentDate: {
        type: "date",
        defaultValue: new Date(),
      },
      completedModules: {
        type: "string", // Store as JSON string
        defaultValue: "[]",
      },
      completedLabs: {
        type: "string", // Store as JSON string
        defaultValue: "[]",
      },
      currentModule: {
        type: "number",
        defaultValue: 1,
      },
      progress: {
        type: "number",
        defaultValue: 0,
      },
      preferredLanguage: {
        type: "string",
        defaultValue: "en",
      },
      usePersonalization: {
        type: "boolean",
        defaultValue: true,
      },
      fullName: {
        type: "string",
        defaultValue: "",
      },
      institution: {
        type: "string",
        defaultValue: "",
      },
      role: {
        type: "string",
        defaultValue: "student",
      },
    },
  },
});