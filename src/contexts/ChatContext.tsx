/**
 * Chat Context
 * Manages chatbot state, conversations, and messages
 */

import React, { createContext, useContext, useState, useEffect, ReactNode } from 'react';
import apiClient from '../api/client';
import { useAuth } from './AuthContext';
import type { ChatConversation, ChatMessage, SendMessageRequest } from '../types';

interface ChatContextType {
  conversations: ChatConversation[];
  currentConversationId: string | null;
  messages: ChatMessage[];
  isLoading: boolean;
  isSending: boolean;
  selectedText: string | null;
  setSelectedText: (text: string | null) => void;
  createConversation: () => Promise<string>;
  selectConversation: (id: string) => Promise<void>;
  sendMessage: (content: string, selectedText?: string) => Promise<void>;
  refreshConversations: () => Promise<void>;
}

const ChatContext = createContext<ChatContextType | undefined>(undefined);

export function ChatProvider({ children }: { children: ReactNode }) {
  const { user } = useAuth();
  const [conversations, setConversations] = useState<ChatConversation[]>([]);
  const [currentConversationId, setCurrentConversationId] = useState<string | null>(null);
  const [messages, setMessages] = useState<ChatMessage[]>([]);
  const [isLoading, setIsLoading] = useState(false);
  const [isSending, setIsSending] = useState(false);
  const [selectedText, setSelectedText] = useState<string | null>(null);

  // Load conversations on mount or user change
  useEffect(() => {
    if (user) {
      refreshConversations();
    } else {
      setConversations([]);
      setCurrentConversationId(null);
      setMessages([]);
    }
  }, [user]);

  const refreshConversations = async () => {
    try {
      const data = await apiClient.getConversations();
      setConversations(data);
    } catch (error) {
      console.error("Failed to load conversations", error);
    }
  };

  const createConversation = async (): Promise<string> => {
    setIsLoading(true);
    try {
      const newConv = await apiClient.createConversation();
      setConversations(prev => [newConv, ...prev]);
      setCurrentConversationId(newConv.id);
      setMessages([]);
      return newConv.id;
    } finally {
      setIsLoading(false);
    }
  };

  const selectConversation = async (id: string) => {
    setCurrentConversationId(id);
    setIsLoading(true);
    try {
      const msgs = await apiClient.getMessages(id);
      setMessages(msgs);
    } catch (error) {
      console.error("Failed to load messages", error);
    } finally {
      setIsLoading(false);
    }
  };

  const sendMessage = async (content: string, selectedText?: string) => {
    if (!currentConversationId) {
        // Create conversation if none exists
        const newId = await createConversation();
        // createConversation sets currentConversationId, but state update might be async
        // so we use newId
        await internalSendMessage(newId, content, selectedText);
    } else {
        await internalSendMessage(currentConversationId, content, selectedText);
    }
  };

  const internalSendMessage = async (convId: string, content: string, selectedText?: string) => {
    setIsSending(true);
    // Optimistic update
    const tempId = Date.now().toString();
    const userMsg: ChatMessage = {
        id: tempId,
        conversation_id: convId,
        role: 'user',
        content: content,
        selected_text: selectedText || null,
        sources: null,
        created_at: new Date().toISOString()
    };
    setMessages(prev => [...prev, userMsg]);

    try {
      const responseMsg = await apiClient.sendMessage(convId, { content, selected_text: selectedText });

      // Replace optimistic message with real one (if we wanted to be strict) or just append response
      // Actually, we should reload messages or append response.
      // Appending response is better.
      setMessages(prev => [...prev, responseMsg]);

      // Refresh conversation list to update message count/order
      refreshConversations();
      // Clear selected text after sending
      setSelectedText(null);
    } catch (error) {
      console.error("Failed to send message", error);
      // Remove optimistic message on error?
      setMessages(prev => prev.filter(m => m.id !== tempId));
      alert("Failed to send message");
    } finally {
      setIsSending(false);
    }
  };

  const value: ChatContextType = {
    conversations,
    currentConversationId,
    messages,
    isLoading,
    isSending,
    selectedText,
    setSelectedText,
    createConversation,
    selectConversation,
    sendMessage,
    refreshConversations,
  };

  return <ChatContext.Provider value={value}>{children}</ChatContext.Provider>;
}

export function useChat() {
  const context = useContext(ChatContext);
  if (context === undefined) {
    throw new Error('useChat must be used within a ChatProvider');
  }
  return context;
}
