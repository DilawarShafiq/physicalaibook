/**
 * API Client for Physical AI Textbook Backend
 * Handles all HTTP requests to FastAPI backend
 */

import type {
  SignupRequest,
  SigninRequest,
  AuthResponse,
  User,
  UserProfile,
  ChatConversation,
  ChatMessage,
  SendMessageRequest,
  LearningPathRecommendations,
  PersonalizedContent,
  APIErrorResponse,
} from '../types';

const API_BASE_URL = process.env.REACT_APP_API_BASE_URL || 'http://localhost:8000/api/v1';

class APIClient {
  private baseURL: string;
  private token: string | null;

  constructor(baseURL: string = API_BASE_URL) {
    this.baseURL = baseURL;
    this.token = localStorage.getItem('access_token');
  }

  private getHeaders(): HeadersInit {
    const headers: HeadersInit = {
      'Content-Type': 'application/json',
    };

    if (this.token) {
      headers['Authorization'] = `Bearer ${this.token}`;
    }

    return headers;
  }

  private async request<T>(
    endpoint: string,
    options: RequestInit = {}
  ): Promise<T> {
    const url = `${this.baseURL}${endpoint}`;
    const headers = this.getHeaders();

    const response = await fetch(url, {
      ...options,
      headers: {
        ...headers,
        ...options.headers,
      },
    });

    if (!response.ok) {
      const errorData: APIErrorResponse = await response.json();
      throw new Error(errorData.error.message || 'API request failed');
    }

    return response.json();
  }

  setToken(token: string) {
    this.token = token;
    localStorage.setItem('access_token', token);
  }

  clearToken() {
    this.token = null;
    localStorage.removeItem('access_token');
  }

  // Auth endpoints
  async signup(data: SignupRequest): Promise<AuthResponse> {
    const response = await this.request<AuthResponse>('/auth/signup', {
      method: 'POST',
      body: JSON.stringify(data),
    });
    this.setToken(response.access_token);
    return response;
  }

  async signin(data: SigninRequest): Promise<AuthResponse> {
    const response = await this.request<AuthResponse>('/auth/signin', {
      method: 'POST',
      body: JSON.stringify(data),
    });
    this.setToken(response.access_token);
    return response;
  }

  async getCurrentUser(): Promise<User> {
    return this.request<User>('/auth/me');
  }

  async updateProfile(data: UserProfile): Promise<User> {
    return this.request<User>('/auth/profile', {
      method: 'PATCH',
      body: JSON.stringify(data),
    });
  }

  // Chat endpoints
  async getConversations(): Promise<ChatConversation[]> {
    return this.request<ChatConversation[]>('/chat/conversations');
  }

  async createConversation(): Promise<ChatConversation> {
    return this.request<ChatConversation>('/chat/conversations', {
      method: 'POST',
    });
  }

  async getMessages(conversationId: string): Promise<ChatMessage[]> {
    return this.request<ChatMessage[]>(`/chat/conversations/${conversationId}/messages`);
  }

  async sendMessage(
    conversationId: string,
    data: SendMessageRequest
  ): Promise<ChatMessage> {
    return this.request<ChatMessage>(
      `/chat/conversations/${conversationId}/messages`,
      {
        method: 'POST',
        body: JSON.stringify(data),
      }
    );
  }

  // Personalization endpoints
  async getRecommendations(): Promise<LearningPathRecommendations> {
    return this.request<LearningPathRecommendations>('/personalization/recommendations');
  }

  async regenerateRecommendations(): Promise<LearningPathRecommendations> {
    return this.request<LearningPathRecommendations>(
      '/personalization/recommendations',
      {
        method: 'POST',
      }
    );
  }

  async personalizeChapter(chapterId: string, data: { content: string, chapter_title: string }): Promise<PersonalizedContent> {
    return this.request<PersonalizedContent>(
      `/personalization/chapters/${chapterId}/personalize`,
      {
        method: 'POST',
        body: JSON.stringify(data),
      }
    );
  }

  async translateChapter(content: string, targetLanguage: string = 'Urdu'): Promise<TranslatedContent> {
    return this.request<TranslatedContent>('/personalization/chapters/translate', {
      method: 'POST',
      body: JSON.stringify({
        content,
        target_language: targetLanguage
      }),
    });
  }
}

// Export singleton instance
export const apiClient = new APIClient();
export default apiClient;
