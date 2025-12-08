import React from 'react';
import { AuthProvider } from '../contexts/AuthContext';
import ChatBot from '../components/ChatBot/ChatBot';

// Default implementation, that you can customize
export default function Root({children}) {
  return (
    <AuthProvider>
      {children}
      <ChatBot />
    </AuthProvider>
  );
}