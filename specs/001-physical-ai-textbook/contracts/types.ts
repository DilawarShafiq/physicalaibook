/**
 * Shared TypeScript Types for Physical AI Textbook
 *
 * This file contains type definitions shared between the frontend (Docusaurus)
 * and backend (FastAPI) API contracts. Keep in sync with openapi.yaml.
 *
 * Generated from: contracts/openapi.yaml
 * Last updated: 2025-12-08
 */

// ============================================================================
// User & Authentication Types
// ============================================================================

export type SoftwareExperience = 'beginner' | 'intermediate' | 'advanced';
export type HardwareExperience = 'none' | 'hobbyist' | 'professional';

export interface UserCreate {
  email: string;
  password: string;
  software_experience: SoftwareExperience;
  hardware_experience: HardwareExperience;
}

export interface UserUpdate {
  software_experience?: SoftwareExperience;
  hardware_experience?: HardwareExperience;
}

export interface User {
  id: string; // UUID
  email: string;
  software_experience: SoftwareExperience;
  hardware_experience: HardwareExperience;
  created_at: string; // ISO 8601 timestamp
  updated_at: string;
  last_login_at: string | null;
  is_active: boolean;
}

export interface AuthTokens {
  access_token: string;
  token_type: 'bearer';
}

export interface SignUpResponse {
  user: User;
  access_token: string;
  token_type: 'bearer';
}

// ============================================================================
// Chat & Conversation Types
// ============================================================================

export type MessageRole = 'user' | 'assistant';

export interface ChatConversation {
  id: string; // UUID
  user_id: string; // UUID
  title: string | null;
  created_at: string;
  expires_at: string; // 24 hours from creation
  last_message_at: string | null;
  message_count: number;
  is_active: boolean;
}

export interface ConversationsListResponse {
  conversations: ChatConversation[];
  total: number;
  limit: number;
  offset: number;
}

export interface SelectedTextContext {
  chapter_id: string;
  section_title: string;
  start_offset: number;
  end_offset: number;
}

export interface ChatMessageCreate {
  content: string; // 1-5000 characters
  selected_text?: string; // For selected text Q&A
  selected_context?: SelectedTextContext;
}

export interface MessageSource {
  chapter_id: string;
  section_title: string;
  chunk_id: string;
  relevance_score: number; // 0.0 - 1.0
}

export interface ChatMessage {
  id: string; // UUID
  conversation_id: string; // UUID
  role: MessageRole;
  content: string;
  selected_text: string | null;
  selected_context: SelectedTextContext | null;
  sources: MessageSource[] | null; // Only for assistant messages
  created_at: string;
  sequence_number: number;
}

export interface MessagesListResponse {
  conversation_id: string;
  messages: ChatMessage[];
}

export interface SendMessageResponse {
  user_message: ChatMessage;
  assistant_message: ChatMessage;
}

// ============================================================================
// Personalization Types
// ============================================================================

export interface RecommendedChapter {
  chapter_id: string; // e.g., "curriculum/module-1/ros2-basics"
  module: string; // e.g., "module-1"
  priority: number; // 1 = highest priority
  reason: string; // Why this chapter was recommended
}

export interface LearningPathRecommendation {
  id: string; // UUID
  user_id: string; // UUID
  recommended_chapters: RecommendedChapter[];
  rationale: string; // Overall explanation of recommendations
  software_experience: SoftwareExperience; // Snapshot at generation time
  hardware_experience: HardwareExperience;
  created_at: string;
  expires_at: string; // 7 days from creation
  is_active: boolean;
}

export interface PersonalizeChapterRequest {
  original_content: string;
}

export interface PersonalizedContentResponse {
  personalized_content: string;
  from_cache: boolean;
  cache_age_hours?: number; // Only present if from_cache = true
}

// ============================================================================
// API Error Types
// ============================================================================

export interface APIError {
  detail: string;
  code?: string;
}

// ============================================================================
// API Client Helper Types
// ============================================================================

/**
 * Configuration for API client
 */
export interface APIConfig {
  baseURL: string; // e.g., "https://api.physicalai-textbook.com/v1"
  timeout?: number; // Request timeout in ms (default: 30000)
}

/**
 * Auth context for frontend state management
 */
export interface AuthContext {
  user: User | null;
  token: string | null;
  isAuthenticated: boolean;
  isLoading: boolean;
}

/**
 * Chat context for frontend state management
 */
export interface ChatContext {
  activeConversationId: string | null;
  conversations: ChatConversation[];
  messages: Map<string, ChatMessage[]>; // conversationId -> messages
  isLoadingMessage: boolean;
}

/**
 * Personalization context for frontend state management
 */
export interface PersonalizationContext {
  recommendations: LearningPathRecommendation | null;
  isLoadingRecommendations: boolean;
  personalizedChapters: Map<string, string>; // chapterId -> personalized content
}

// ============================================================================
// Utility Types
// ============================================================================

/**
 * Standard paginated response wrapper
 */
export interface PaginatedResponse<T> {
  items: T[];
  total: number;
  limit: number;
  offset: number;
}

/**
 * API request options
 */
export interface RequestOptions {
  method?: 'GET' | 'POST' | 'PATCH' | 'DELETE';
  headers?: Record<string, string>;
  body?: unknown;
  params?: Record<string, string | number | boolean>;
}

/**
 * Health check response
 */
export interface HealthResponse {
  status: 'healthy' | 'unhealthy';
  version: string;
  timestamp: string;
}
