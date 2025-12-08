import React, { useEffect, useRef } from 'react';
import type { ChatMessage } from '../../types';
import ReactMarkdown from 'react-markdown';

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
                <div style={{ textAlign: 'center', color: '#6c757d', marginTop: '2rem' }}>
                    <div style={{ fontSize: '3rem', marginBottom: '1rem' }}>📚</div>
                    <p style={{ fontSize: '1.1rem', fontWeight: '500' }}>Ask a question about the textbook!</p>
                    <p style={{ fontSize: '0.9rem', color: '#888' }}>I can help explain concepts, provide examples, and guide your learning.</p>
                </div>
            )}

            {messages.map((msg) => (
                <div
                    key={msg.id}
                    style={{
                        alignSelf: msg.role === 'user' ? 'flex-end' : 'flex-start',
                        maxWidth: '90%',
                        background: msg.role === 'user' ? 'linear-gradient(135deg, #1a73e8, #0d6efd)' : '#f8f9fa',
                        color: msg.role === 'user' ? '#fff' : '#212529',
                        padding: '1rem',
                        borderRadius: '16px',
                        borderBottomRightRadius: msg.role === 'user' ? '4px' : '16px',
                        borderBottomLeftRadius: msg.role === 'assistant' ? '4px' : '16px',
                        boxShadow: '0 2px 8px rgba(0,0,0,0.08)',
                    }}
                >
                    <div style={{ display: 'flex', alignItems: 'flex-start', gap: '0.5rem' }}>
                        {msg.role === 'assistant' && (
                            <div style={{
                                fontSize: '1.2rem',
                                minWidth: '24px',
                                display: 'flex',
                                alignItems: 'center'
                            }}>
                                🤖
                            </div>
                        )}
                        <div style={{ flex: 1 }}>
                            <ReactMarkdown
                                components={{
                                    p: ({node, ...props}) => <p style={{ marginBottom: '0.75rem', lineHeight: '1.5' }} {...props} />,
                                    ul: ({node, ...props}) => <ul style={{ paddingLeft: '1.5rem', marginBottom: '0.75rem' }} {...props} />,
                                    ol: ({node, ...props}) => <ol style={{ paddingLeft: '1.5rem', marginBottom: '0.75rem' }} {...props} />,
                                    li: ({node, ...props}) => <li style={{ marginBottom: '0.25rem' }} {...props} />,
                                    code: ({node, ...props}) => <code style={{
                                        background: msg.role === 'user' ? 'rgba(255,255,255,0.2)' : '#e9ecef',
                                        padding: '0.2rem 0.4rem',
                                        borderRadius: '4px',
                                        fontSize: '0.875em',
                                        fontFamily: 'monospace'
                                    }} {...props} />,
                                    pre: ({node, ...props}) => <pre style={{
                                        background: msg.role === 'user' ? 'rgba(255,255,255,0.2)' : '#f8f9fa',
                                        padding: '0.75rem',
                                        borderRadius: '8px',
                                        overflowX: 'auto',
                                        margin: '0.75rem 0',
                                        fontFamily: 'monospace',
                                        fontSize: '0.875em'
                                    }} {...props} />,
                                    strong: ({node, ...props}) => <strong style={{ fontWeight: '600' }} {...props} />,
                                    em: ({node, ...props}) => <em style={{ fontStyle: 'italic' }} {...props} />,
                                    blockquote: ({node, ...props}) => <blockquote style={{
                                        borderLeft: '3px solid #1a73e8',
                                        paddingLeft: '1rem',
                                        margin: '0.75rem 0',
                                        color: 'inherit',
                                        fontStyle: 'italic'
                                    }} {...props} />,
                                    a: ({node, ...props}) => <a style={{
                                        color: msg.role === 'user' ? '#b3d9ff' : '#1a73e8',
                                        textDecoration: 'underline'
                                    }} {...props} />
                                }}
                            >
                                {msg.content}
                            </ReactMarkdown>

                            {msg.selected_text && (
                                <div style={{
                                    marginTop: '0.75rem',
                                    padding: '0.5rem',
                                    background: msg.role === 'user' ? 'rgba(255,255,255,0.15)' : '#e9ecef',
                                    borderLeft: '3px solid #1a73e8',
                                    borderRadius: '0 4px 4px 0',
                                    fontStyle: 'italic',
                                    fontSize: '0.9em'
                                }}>
                                    <small style={{ fontWeight: '600', display: 'block', marginBottom: '0.25rem' }}>Referenced text:</small>
                                    "{msg.selected_text.substring(0, 200)}{msg.selected_text.length > 200 ? '...' : ''}"
                                </div>
                            )}

                            {msg.sources && msg.sources.length > 0 && (
                                <div style={{
                                    marginTop: '0.75rem',
                                    fontSize: '0.8rem',
                                    borderTop: msg.role === 'user' ? '1px solid rgba(255,255,255,0.2)' : '1px solid #dee2e6',
                                    paddingTop: '0.5rem'
                                }}>
                                    <strong style={{ display: 'block', marginBottom: '0.25rem' }}>📚 Sources:</strong>
                                    <ul style={{ paddingLeft: '1rem', margin: '0.25rem 0', fontSize: '0.85rem' }}>
                                        {msg.sources.map((src, idx) => (
                                            <li key={idx} style={{ marginBottom: '0.2rem' }}>
                                                {src.metadata?.path ? (
                                                    <span title={src.content.substring(0, 100) + '...'} style={{
                                                        wordBreak: 'break-word'
                                                    }}>
                                                        {src.metadata.path}
                                                    </span>
                                                ) : (
                                                    <span>Textbook content</span>
                                                )}
                                            </li>
                                        ))}
                                    </ul>
                                </div>
                            )}
                        </div>
                        {msg.role === 'user' && (
                            <div style={{
                                fontSize: '1.2rem',
                                minWidth: '24px',
                                display: 'flex',
                                alignItems: 'center'
                            }}>
                                👤
                            </div>
                        )}
                    </div>
                </div>
            ))}

            {isLoading && (
                <div style={{
                    alignSelf: 'flex-start',
                    background: '#f8f9fa',
                    padding: '1rem',
                    borderRadius: '16px',
                    borderBottomRightRadius: '4px',
                    boxShadow: '0 2px 8px rgba(0,0,0,0.08)',
                    display: 'flex',
                    alignItems: 'center',
                    gap: '0.5rem'
                }}>
                    <div>🤖</div>
                    <div className="typing-indicator" style={{ display: 'flex', alignItems: 'center', gap: '0.25rem' }}>
                        <span>Thinking</span>
                        <span>.</span>
                        <span>.</span>
                        <span>.</span>
                    </div>
                </div>
            )}

            <div ref={bottomRef} />
        </div>
    );
}
