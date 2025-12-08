import React from 'react';
import { AuthProvider } from '../contexts/AuthContext';
import { ChatProvider } from '../contexts/ChatContext';
import ChatBot from '../components/ChatBot/ChatBot';
import TextSelectionHandler from '../components/TextSelectionHandler';

// Default implementation, that you can customize
export default function Root({children}) {
  return (
    <AuthProvider>
      <ChatProvider>
        {children}
        <ChatBot />
        <TextSelectionHandler />
      </ChatProvider>
    </AuthProvider>
  );
}