/**
 * Better Auth Client
 * Frontend client for authentication
 */

import { createAuthClient } from "better-auth/react";

export const authClient = createAuthClient({
  baseURL: process.env.BETTER_AUTH_URL || "http://localhost:3001",
});

export const {
  signIn,
  signUp,
  signOut,
  useSession,
  $fetch,
} = authClient;

// Helper functions for auth operations
export const authHelpers = {
  // Sign up with student details
  async signUpStudent(data: {
    email: string;
    password: string;
    name: string;
    softwareExperience: "beginner" | "intermediate" | "advanced";
    hardwareExperience: "none" | "hobbyist" | "professional";
  }) {
    return await signUp.email({
      email: data.email,
      password: data.password,
      name: data.name,
      callbackURL: "/curriculum/introduction",
    }, {
      onRequest: (context) => {
        // Add custom fields to request
        context.body = {
          ...context.body,
          softwareExperience: data.softwareExperience,
          hardwareExperience: data.hardwareExperience,
        };
      },
    });
  },

  // Sign in
  async signInStudent(email: string, password: string) {
    return await signIn.email({
      email,
      password,
      callbackURL: "/curriculum/introduction",
    });
  },

  // Update student progress
  async updateProgress(moduleId: number, progress: number) {
    return await $fetch("/api/auth/update-progress", {
      method: "POST",
      body: JSON.stringify({ moduleId, progress }),
    });
  },

  // Mark module as completed
  async completeModule(moduleId: number) {
    return await $fetch("/api/auth/complete-module", {
      method: "POST",
      body: JSON.stringify({ moduleId }),
    });
  },

  // Mark lab as completed
  async completeLab(labId: string) {
    return await $fetch("/api/auth/complete-lab", {
      method: "POST",
      body: JSON.stringify({ labId }),
    });
  },
};
