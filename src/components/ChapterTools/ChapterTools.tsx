import React, { useState, useEffect } from 'react';
import { useAuth } from '../../contexts/AuthContext';
import apiClient from '../../api/client';
import styles from './ChapterTools.module.css';

interface ChapterToolsProps {
  chapterId?: string;
  title?: string;
}

export default function ChapterTools({ chapterId = 'unknown', title }: ChapterToolsProps) {
  const { user, isAuthenticated } = useAuth();
  const [loading, setLoading] = useState(false);
  const [originalContent, setOriginalContent] = useState<string | null>(null);
  const [mode, setMode] = useState<'original' | 'personalized' | 'translated'>('original');
  const [error, setError] = useState<string | null>(null);
  const [targetLanguage, setTargetLanguage] = useState<string>('Urdu');

  useEffect(() => {
    // Get the original content when component mounts
    const article = document.querySelector('article');
    if (article) {
      // Store the original HTML content
      setOriginalContent(article.innerHTML);
    }
  }, []);

  if (!isAuthenticated) return null;

  const handlePersonalize = async () => {
    try {
      setLoading(true);
      setError(null);

      if (!originalContent) {
        throw new Error('Could not extract content to personalize.');
      }

      const response = await apiClient.personalizeChapter(chapterId, {
        content: originalContent,
        chapter_title: title || chapterId
      });

      // Replace the article content with personalized content
      const article = document.querySelector('article');
      if (article) {
        article.innerHTML = response.personalized_content;
      }
      setMode('personalized');
    } catch (err: any) {
      setError(err.message || 'Failed to personalize content.');
    } finally {
      setLoading(false);
    }
  };

  const handleTranslate = async () => {
    try {
      setLoading(true);
      setError(null);

      if (!originalContent) {
        throw new Error('Could not extract content to translate.');
      }

      const response = await apiClient.translateChapter(originalContent, targetLanguage);

      // Replace the article content with translated content
      const article = document.querySelector('article');
      if (article) {
        article.innerHTML = response.translated_content;
      }
      setMode('translated');
    } catch (err: any) {
      setError(err.message || 'Failed to translate content.');
    } finally {
      setLoading(false);
    }
  };

  const handleReset = () => {
    // Restore the original content
    const article = document.querySelector('article');
    if (article && originalContent) {
      article.innerHTML = originalContent;
    }
    setMode('original');
  };

  return (
    <div className={styles.container}>
      <div className={styles.controls}>
        <div className={styles.header}>
          <h4 className={styles.title}>📚 Chapter Tools</h4>
          <span className={styles.greeting}>Hi, {user?.email.split('@')[0]}! 👋</span>
        </div>

        <div className={styles.buttons}>
          <button
            onClick={handlePersonalize}
            disabled={loading || mode === 'personalized'}
            className={`${styles.button} ${styles.personalizeBtn}`}
            title="Personalize content based on your experience level"
          >
            <span className={styles.buttonIcon}>🔧</span>
            {loading && mode === 'personalized' ? 'Personalizing...' : 'Personalize for Me'}
          </button>

          <div className={styles.translateContainer}>
            <select
              value={targetLanguage}
              onChange={(e) => setTargetLanguage(e.target.value)}
              className={styles.languageSelect}
              disabled={loading}
            >
              <option value="Urdu">🇵🇰 Translate to Urdu</option>
              <option value="Chinese">🇨🇳 Translate to Chinese</option>
              <option value="Spanish">🇪🇸 Translate to Spanish</option>
              <option value="French">🇫🇷 Translate to French</option>
              <option value="Arabic">🇸🇦 Translate to Arabic</option>
            </select>
            <button
              onClick={handleTranslate}
              disabled={loading || mode === 'translated'}
              className={`${styles.button} ${styles.translateBtn}`}
              title={`Translate content to ${targetLanguage}`}
            >
              <span className={styles.buttonIcon}>🌐</span>
              {loading && mode === 'translated' ? 'Translating...' : 'Translate'}
            </button>
          </div>

          {(mode === 'personalized' || mode === 'translated') && (
            <button
              onClick={handleReset}
              className={styles.resetButton}
              title="Show original content"
            >
              <span className={styles.buttonIcon}>🔄</span>
              Show Original
            </button>
          )}
        </div>
      </div>

      {error && (
        <div className={styles.error}>
          <span className={styles.errorIcon}>⚠️</span>
          {error}
        </div>
      )}
    </div>
  );
}