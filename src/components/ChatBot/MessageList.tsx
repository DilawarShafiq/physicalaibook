import React, { useEffect, useRef } from 'react';
import type { ChatMessage } from '../../types';
import ReactMarkdown from 'react-markdown'; // Docusaurus might not have this by default, but let's see. 
// If not, just render text. Docusaurus has MDX.
// I'll render plain text or simple HTML for now to avoid dep issues if I can't check.
// Actually, I can use simple rendering.

interface MessageListProps {
    messages: ChatMessage[];
    isLoading?: boolean;
}

export default function MessageList({ messages, isLoading }: MessageListProps) {
    const bottomRef = useRef<HTMLDivElement>(null);

    useEffect(() => {
        bottomRef.current?.scrollIntoView({ behavior: 'smooth' });
    }, [messages, isLoading]);

    return (
        <div style={{ flex: 1, overflowY: 'auto', padding: '1rem', display: 'flex', flexDirection: 'column', gap: '1rem' }}>
            {messages.length === 0 && (
                <div style={{ textAlign: 'center', color: '#888', marginTop: '2rem' }}>
                    <p>Ask a question about the textbook!</p>
                </div>
            )}
            
            {messages.map((msg) => (
                <div 
                    key={msg.id} 
                    style={{ 
                        alignSelf: msg.role === 'user' ? 'flex-end' : 'flex-start',
                        maxWidth: '85%',
                        background: msg.role === 'user' ? '#007bff' : '#f0f2f5',
                        color: msg.role === 'user' ? '#fff' : '#000',
                        padding: '0.75rem',
                        borderRadius: '12px',
                        borderBottomRightRadius: msg.role === 'user' ? '2px' : '12px',
                        borderBottomLeftRadius: msg.role === 'assistant' ? '2px' : '12px',
                    }}
                >
                    <div style={{ whiteSpace: 'pre-wrap', fontSize: '0.9rem' }}>
                        {msg.content}
                    </div>
                    
                    {msg.sources && msg.sources.length > 0 && (
                        <div style={{ marginTop: '0.5rem', fontSize: '0.75rem', borderTop: '1px solid rgba(0,0,0,0.1)', paddingTop: '0.25rem' }}>
                            <strong>Sources:</strong>
                            <ul style={{ paddingLeft: '1rem', margin: '0.25rem 0' }}>
                                {msg.sources.map((src, idx) => (
                                    <li key={idx}>
                                        {src.metadata.path ? (
                                            <span title={src.content}>{src.metadata.path}</span>
                                        ) : 'Textbook'}
                                    </li>
                                ))}
                            </ul>
                        </div>
                    )}
                </div>
            ))}
            
            {isLoading && (
                <div style={{ alignSelf: 'flex-start', background: '#f0f2f5', padding: '0.75rem', borderRadius: '12px' }}>
                    <div className="typing-indicator">Thinking...</div>
                </div>
            )}
            
            <div ref={bottomRef} />
        </div>
    );
}
