import React, { useState } from 'react';
import { useChat, ChatProvider } from '../../contexts/ChatContext';
import { useAuth } from '../../contexts/AuthContext';
import ChatInput from './ChatInput';
import MessageList from './MessageList';
import BrowserOnly from '@docusaurus/BrowserOnly';

function ChatBotContent() {
    const { user } = useAuth();
    const { messages, sendMessage, isSending, createConversation, refreshConversations } = useChat();
    const [isOpen, setIsOpen] = useState(false);

    if (!user) {
        // Only show if logged in? Or prompt to login.
        // Spec says US3 requires auth.
        // I'll show a teaser button that asks to login.
        if (!isOpen) {
             return (
                <button 
                    onClick={() => alert("Please sign in to use the AI Assistant")}
                    style={{
                        position: 'fixed',
                        bottom: '20px',
                        right: '20px',
                        zIndex: 1000,
                        width: '50px',
                        height: '50px',
                        borderRadius: '25px',
                        background: '#007bff',
                        color: '#fff',
                        border: 'none',
                        cursor: 'pointer',
                        boxShadow: '0 2px 10px rgba(0,0,0,0.2)'
                    }}
                >
                    💬
                </button>
            );
        }
        return null;
    }

    if (!isOpen) {
        return (
            <button 
                onClick={() => setIsOpen(true)}
                style={{
                    position: 'fixed',
                    bottom: '20px',
                    right: '20px',
                    zIndex: 1000,
                    width: '60px',
                    height: '60px',
                    borderRadius: '30px',
                    background: '#007bff',
                    color: '#fff',
                    border: 'none',
                    cursor: 'pointer',
                    boxShadow: '0 4px 15px rgba(0,0,0,0.2)',
                    fontSize: '1.5rem',
                    display: 'flex',
                    alignItems: 'center',
                    justifyContent: 'center'
                }}
            >
                🤖
            </button>
        );
    }

    return (
        <div style={{
            position: 'fixed',
            bottom: '20px',
            right: '20px',
            zIndex: 1000,
            width: '350px',
            height: '500px',
            background: '#fff',
            borderRadius: '12px',
            boxShadow: '0 5px 25px rgba(0,0,0,0.2)',
            display: 'flex',
            flexDirection: 'column',
            overflow: 'hidden',
            border: '1px solid #ddd'
        }}>
            <div style={{ 
                padding: '10px 15px', 
                background: '#007bff', 
                color: '#fff', 
                display: 'flex', 
                justifyContent: 'space-between',
                alignItems: 'center'
            }}>
                <h3 style={{ margin: 0, fontSize: '1rem' }}>AI Assistant</h3>
                <div style={{ display: 'flex', gap: '10px' }}>
                    <button 
                        onClick={createConversation} 
                        title="New Chat"
                        style={{ background: 'none', border: 'none', color: '#fff', cursor: 'pointer', fontSize: '1.2rem' }}
                    >
                        +
                    </button>
                    <button 
                        onClick={() => setIsOpen(false)} 
                        title="Close"
                        style={{ background: 'none', border: 'none', color: '#fff', cursor: 'pointer', fontSize: '1.2rem' }}
                    >
                        ×
                    </button>
                </div>
            </div>
            
            <MessageList messages={messages} isLoading={isSending} />
            
            <ChatInput onSend={(text) => sendMessage(text)} disabled={isSending} />
        </div>
    );
}

export default function ChatBot() {
    return (
        <BrowserOnly>
            {() => (
                <ChatProvider>
                    <ChatBotContent />
                </ChatProvider>
            )}
        </BrowserOnly>
    );
}
