import React from 'react';
import { AuthProvider } from '../contexts/AuthContext';
import ChatBot from '../components/ChatBot/ChatBot';
import TextSelectionHandler from '../components/TextSelectionHandler';

// Default implementation, that you can customize
export default function Root({children}) {
  return (
    <AuthProvider>
      {children}
      <ChatBot />
      <TextSelectionHandler />
    </AuthProvider>
  );
}