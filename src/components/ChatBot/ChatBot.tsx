import React, { useState } from 'react';
import { useChat, ChatProvider } from '../../contexts/ChatContext';
import { useAuth } from '../../contexts/AuthContext';
import ChatInput from './ChatInput';
import MessageList from './MessageList';
import BrowserOnly from '@docusaurus/BrowserOnly';

function ChatBotContent() {
    const { user } = useAuth();
    const { messages, sendMessage, isSending, createConversation, refreshConversations, selectedText } = useChat();
    const [isOpen, setIsOpen] = useState(false);

    if (!user) {
        // Only show if logged in? Or prompt to login.
        // Spec says US3 requires auth.
        // I'll show a teaser button that asks to login.
        if (!isOpen) {
             return (
                <button
                    onClick={() => alert("Please sign in to use the AI Assistant")}
                    className="chat-widget-container"
                    style={{
                        position: 'fixed',
                        bottom: '20px',
                        right: '20px',
                        zIndex: 1000,
                        width: '60px',
                        height: '60px',
                        borderRadius: '30px',
                        background: 'linear-gradient(135deg, #1a73e8, #34a853)',
                        color: '#fff',
                        border: 'none',
                        cursor: 'pointer',
                        boxShadow: '0 4px 20px rgba(26, 115, 232, 0.4)',
                        fontSize: '1.2rem',
                        display: 'flex',
                        alignItems: 'center',
                        justifyContent: 'center',
                        transition: 'all 0.3s ease',
                    }}
                    onMouseEnter={(e) => {
                        e.currentTarget.style.transform = 'scale(1.1)';
                    }}
                    onMouseLeave={(e) => {
                        e.currentTarget.style.transform = 'scale(1)';
                    }}
                >
                    🤖
                </button>
            );
        }
        return null;
    }

    if (!isOpen) {
        return (
            <button
                onClick={() => setIsOpen(true)}
                className="chat-widget-container"
                style={{
                    position: 'fixed',
                    bottom: '20px',
                    right: '20px',
                    zIndex: 1000,
                    width: '60px',
                    height: '60px',
                    borderRadius: '30px',
                    background: 'linear-gradient(135deg, #1a73e8, #34a853)',
                    color: '#fff',
                    border: 'none',
                    cursor: 'pointer',
                    boxShadow: '0 6px 20px rgba(26, 115, 232, 0.4)',
                    fontSize: '1.2rem',
                    display: 'flex',
                    alignItems: 'center',
                    justifyContent: 'center',
                    transition: 'all 0.3s ease',
                    ...(selectedText ? { transform: 'scale(1.1)', boxShadow: '0 8px 25px rgba(26, 115, 232, 0.5)' } : {}),
                }}
                onMouseEnter={(e) => {
                    e.currentTarget.style.transform = 'scale(1.1)';
                }}
                onMouseLeave={(e) => {
                    e.currentTarget.style.transform = 'scale(1)';
                }}
            >
                {selectedText ? '💬' : '🤖'}
            </button>
        );
    }

    return (
        <div style={{
            position: 'fixed',
            bottom: '20px',
            right: '20px',
            zIndex: 1000,
            width: '420px',
            height: '600px',
            background: '#fff',
            borderRadius: '20px',
            boxShadow: '0 15px 40px rgba(0,0,0,0.2)',
            display: 'flex',
            flexDirection: 'column',
            overflow: 'hidden',
            border: '1px solid var(--ifm-toc-border-color)',
            transition: 'all 0.3s ease',
        }}>
            <div style={{
                padding: '18px 20px',
                background: 'linear-gradient(135deg, var(--ifm-color-primary), #34a853)',
                color: '#fff',
                display: 'flex',
                justifyContent: 'space-between',
                alignItems: 'center'
            }}>
                <div style={{ display: 'flex', alignItems: 'center', gap: '12px' }}>
                    <span style={{ fontSize: '1.6rem' }}>🤖</span>
                    <div>
                        <h3 style={{ margin: 0, fontSize: '1.2rem', fontWeight: '600' }}>AI Assistant</h3>
                        {selectedText && (
                            <div
                                style={{
                                    fontSize: '0.75rem',
                                    background: 'rgba(255,255,255,0.2)',
                                    padding: '0.2rem 0.5rem',
                                    borderRadius: '12px',
                                    marginTop: '0.25rem',
                                    maxWidth: '250px',
                                    overflow: 'hidden',
                                    textOverflow: 'ellipsis',
                                    whiteSpace: 'nowrap',
                                    cursor: 'pointer'
                                }}
                                title={`Selected: ${selectedText}`}
                                onClick={() => sendMessage(`Explain this: "${selectedText}"`, selectedText)}
                            >
                                <span style={{ fontWeight: '500' }}>💡</span> Selected: "{selectedText.substring(0, 30)}{selectedText.length > 30 ? '...' : ''}"
                            </div>
                        )}
                    </div>
                </div>
                <div style={{ display: 'flex', gap: '12px' }}>
                    <button
                        onClick={createConversation}
                        title="New Chat"
                        style={{
                            background: 'rgba(255, 255, 255, 0.2)',
                            border: 'none',
                            color: '#fff',
                            cursor: 'pointer',
                            borderRadius: '50%',
                            width: '36px',
                            height: '36px',
                            display: 'flex',
                            alignItems: 'center',
                            justifyContent: 'center',
                            fontSize: '1.2rem',
                            transition: 'background 0.2s ease',
                            fontWeight: 'bold'
                        }}
                        onMouseEnter={(e) => {
                            e.currentTarget.style.background = 'rgba(255, 255, 255, 0.3)';
                        }}
                        onMouseLeave={(e) => {
                            e.currentTarget.style.background = 'rgba(255, 255, 255, 0.2)';
                        }}
                    >
                        +
                    </button>
                    <button
                        onClick={() => setIsOpen(false)}
                        title="Close"
                        style={{
                            background: 'rgba(255, 255, 255, 0.2)',
                            border: 'none',
                            color: '#fff',
                            cursor: 'pointer',
                            borderRadius: '50%',
                            width: '36px',
                            height: '36px',
                            display: 'flex',
                            alignItems: 'center',
                            justifyContent: 'center',
                            fontSize: '1.3rem',
                            transition: 'background 0.2s ease'
                        }}
                        onMouseEnter={(e) => {
                            e.currentTarget.style.background = 'rgba(255, 255, 255, 0.3)';
                        }}
                        onMouseLeave={(e) => {
                            e.currentTarget.style.background = 'rgba(255, 255, 255, 0.2)';
                        }}
                    >
                        ×
                    </button>
                </div>
            </div>

            <MessageList messages={messages} isLoading={isSending} />

            <ChatInput
                onSend={(text) => {
                    // If there's selected text and the input is empty, ask about the selected text
                    if (!text.trim() && selectedText) {
                        sendMessage(`Explain this: "${selectedText}"`, selectedText);
                    } else {
                        sendMessage(text, selectedText);
                    }
                }}
                disabled={isSending}
            />
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
