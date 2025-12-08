/**
 * TypeScript type definitions for the Physical AI Textbook
 * Shared types between frontend and backend API
 */

// User types
export type SoftwareExperience = 'beginner' | 'intermediate' | 'advanced';
export type HardwareExperience = 'none' | 'hobbyist' | 'professional';

export interface User {
  id: string;
  email: string;
  software_experience: SoftwareExperience | null;
  hardware_experience: HardwareExperience | null;
  created_at: string;
}

export interface UserProfile {
  software_experience: SoftwareExperience;
  hardware_experience: HardwareExperience;
}

// Auth types
export interface SignupRequest {
  email: string;
  password: string;
  software_experience: SoftwareExperience;
  hardware_experience: HardwareExperience;
}

export interface SigninRequest {
  email: string;
  password: string;
}

export interface AuthResponse {
  access_token: string;
  token_type: string;
  user: User;
}

// Chat types
export type MessageRole = 'user' | 'assistant';

export interface MessageSource {
  content: string;
  metadata: {
    chapter_id?: string;
    module_id?: string;
    section_title?: string;
  };
}

export interface ChatMessage {
  id: string;
  conversation_id: string;
  role: MessageRole;
  content: string;
  sources: MessageSource[] | null;
  selected_text: string | null;
  created_at: string;
}

export interface ChatConversation {
  id: string;
  user_id: string;
  created_at: string;
  expires_at: string;
  message_count: number;
}

export interface SendMessageRequest {
  content: string;
  selected_text?: string;
}

// Personalization types
export interface LearningPathItem {
  module_id: string;
  title: string;
  reason: string;
  difficulty: 'beginner' | 'intermediate' | 'advanced';
}

export interface LearningPathRecommendations {
  recommendations: LearningPathItem[];
  created_at: string;
}

// Translation types
export interface TranslateChapterRequest {
  content: string;
  target_language: string;
  chapter_title?: string;
}

export interface TranslatedContent {
  original_content: string;
  translated_content: string;
  target_language: string;
  chapter_id: string;
}

export interface PersonalizeChapterRequest {
  content: string;
  chapter_title: string;
}

export interface PersonalizedContent {
  chapter_id: string;
  personalized_content: string;
  created_at: string;
}

// API Error types
export interface APIError {
  code: string;
  message: string;
  details?: any;
  path?: string;
}

export interface APIErrorResponse {
  error: APIError;
}
