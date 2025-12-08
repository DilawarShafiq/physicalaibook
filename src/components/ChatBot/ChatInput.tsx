import React, { useState, useRef, useEffect } from 'react';

interface ChatInputProps {
    onSend: (content: string) => void;
    disabled?: boolean;
}

export default function ChatInput({ onSend, disabled }: ChatInputProps) {
    const [text, setText] = useState('');
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
    
    return (
        <div style={{ borderTop: '1px solid #ddd', padding: '0.5rem', background: '#f5f5f5' }}>
            <form onSubmit={handleSubmit} style={{ display: 'flex', gap: '0.5rem' }}>
                <textarea 
                    ref={textareaRef}
                    value={text} 
                    onChange={e => setText(e.target.value)} 
                    onKeyDown={handleKeyDown}
                    placeholder="Ask a question..."
                    disabled={disabled}
                    style={{ 
                        flex: 1, 
                        resize: 'none', 
                        height: '50px', 
                        padding: '0.5rem', 
                        borderRadius: '4px',
                        border: '1px solid #ccc',
                        fontFamily: 'inherit'
                    }}
                />
                <button 
                    type="submit" 
                    disabled={disabled || !text.trim()}
                    className="button button--primary button--sm"
                    style={{ alignSelf: 'flex-end', height: '50px' }}
                >
                    Send
                </button>
            </form>
        </div>
    );
}
