import React, { useEffect } from 'react';
import { useChat } from '../contexts/ChatContext';

const TextSelectionHandler: React.FC = () => {
  const { setSelectedText } = useChat();

  useEffect(() => {
    const handleSelection = () => {
      const selection = window.getSelection();
      if (selection && selection.toString().trim() !== '') {
        const selectedText = selection.toString().trim();
        if (selectedText.length > 0 && selectedText.length < 1000) { // Reasonable length check
          setSelectedText(selectedText);
        }
      } else {
        setSelectedText(null);
      }
    };

    document.addEventListener('mouseup', handleSelection);
    document.addEventListener('keyup', handleSelection);

    return () => {
      document.removeEventListener('mouseup', handleSelection);
      document.removeEventListener('keyup', handleSelection);
    };
  }, [setSelectedText]);

  return null;
};

export default TextSelectionHandler;