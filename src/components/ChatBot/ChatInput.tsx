import React, { useState, useRef, useEffect } from 'react';

interface ChatInputProps {
    onSend: (content: string) => void;
    disabled?: boolean;
}

export default function ChatInput({ onSend, disabled }: ChatInputProps) {
    const [text, setText] = useState('');
    const [isFocused, setIsFocused] = useState(false);
    const textareaRef = useRef<HTMLTextAreaElement>(null);

    const handleSubmit = (e?: React.FormEvent) => {
        if (e) e.preventDefault();
        if (text.trim() && !disabled) {
            onSend(text);
            setText('');
        }
    };

    const handleKeyDown = (e: React.KeyboardEvent) => {
        if (e.key === 'Enter' && !e.shiftKey) {
            e.preventDefault();
            handleSubmit();
        }
    };

    const handleInputFocus = () => {
        setIsFocused(true);
    };

    const handleInputBlur = () => {
        setIsFocused(false);
    };

    // Auto-resize textarea based on content
    useEffect(() => {
        if (textareaRef.current) {
            textareaRef.current.style.height = 'auto';
            textareaRef.current.style.height = `${Math.min(textareaRef.current.scrollHeight, 150)}px`;
        }
    }, [text]);

    return (
        <div style={{
            borderTop: '1px solid var(--ifm-toc-border-color)',
            padding: '1rem',
            background: 'var(--ifm-background-surface-color)',
            boxShadow: '0 -2px 10px rgba(0,0,0,0.05)'
        }}>
            <form
                onSubmit={handleSubmit}
                style={{
                    display: 'flex',
                    gap: '0.75rem',
                    alignItems: 'flex-end'
                }}
            >
                <div style={{
                    flex: 1,
                    position: 'relative'
                }}>
                    <textarea
                        ref={textareaRef}
                        value={text}
                        onChange={e => setText(e.target.value)}
                        onKeyDown={handleKeyDown}
                        onFocus={handleInputFocus}
                        onBlur={handleInputBlur}
                        placeholder="Ask about the textbook content..."
                        disabled={disabled}
                        style={{
                            width: '100%',
                            resize: 'none',
                            minHeight: '50px',
                            maxHeight: '150px',
                            padding: '0.75rem 4rem 0.75rem 1rem', // Extra padding on right for send button
                            borderRadius: '24px',
                            border: `2px solid ${isFocused ? 'var(--ifm-color-primary)' : 'var(--ifm-toc-border-color)'}`,
                            fontFamily: 'inherit',
                            fontSize: '0.9rem',
                            background: 'var(--ifm-background-color)',
                            outline: 'none',
                            transition: 'border-color 0.2s ease',
                        }}
                    />
                    <div style={{
                        position: 'absolute',
                        right: '1rem',
                        bottom: '0.5rem',
                        display: 'flex',
                        alignItems: 'center',
                        gap: '0.25rem',
                        color: '#888',
                        fontSize: '0.8rem'
                    }}>
                        <span style={{ fontSize: '0.7rem' }}>Press ⏎ for new line, Shift+⏎ to send</span>
                    </div>
                </div>

                <button
                    type="submit"
                    disabled={disabled || !text.trim()}
                    className="button button--primary"
                    style={{
                        borderRadius: '24px',
                        padding: '0.75rem 1.5rem',
                        height: 'fit-content',
                        background: 'linear-gradient(135deg, var(--ifm-color-primary), #34a853)',
                        border: 'none',
                        fontWeight: '500',
                        boxShadow: '0 2px 8px rgba(26, 115, 232, 0.3)',
                        opacity: disabled || !text.trim() ? 0.6 : 1,
                        cursor: (disabled || !text.trim()) ? 'not-allowed' : 'pointer',
                        transition: 'all 0.2s ease',
                    }}
                    onMouseEnter={(e) => {
                        if (!(disabled || !text.trim())) {
                            e.currentTarget.style.transform = 'translateY(-2px)';
                            e.currentTarget.style.boxShadow = '0 4px 12px rgba(26, 115, 232, 0.4)';
                        }
                    }}
                    onMouseLeave={(e) => {
                        e.currentTarget.style.transform = 'translateY(0)';
                        e.currentTarget.style.boxShadow = '0 2px 8px rgba(26, 115, 232, 0.3)';
                    }}
                >
                    Send
                    <span style={{ marginLeft: '0.5rem' }}>→</span>
                </button>
            </form>

            <div style={{
                marginTop: '0.5rem',
                fontSize: '0.8rem',
                color: '#6c757d',
                textAlign: 'center'
            }}>
                <span>🤖 AI Assistant - Ask questions about the textbook content</span>
            </div>
        </div>
    );
}
