/**
 * Chapter Tools Component
 * Provides personalization and translation buttons for each chapter
 */

import React, { useState } from 'react';
import { useSession } from '../lib/auth-client';
import { apiClient } from '../api/client';
import styles from './ChapterTools.module.css';

interface ChapterToolsProps {
  chapterId: string;
  title: string;
}

export default function ChapterTools({ chapterId, title }: ChapterToolsProps) {
  const { data: session, isPending } = useSession();
  const [isPersonalizing, setIsPersonalizing] = useState(false);
  const [isTranslating, setIsTranslating] = useState(false);
  const [personalizedContent, setPersonalizedContent] = useState<string | null>(null);
  const [translatedContent, setTranslatedContent] = useState<string | null>(null);
  const [showOriginal, setShowOriginal] = useState(true);

  const handlePersonalize = async () => {
    if (!session) {
      alert('Please sign in to personalize content');
      return;
    }

    setIsPersonalizing(true);
    try {
      // Get the chapter content from the current page
      const contentElement = document.querySelector('article');
      const content = contentElement?.innerText || '';

      const result = await apiClient.personalizeChapter(title, content);
      setPersonalizedContent(result.personalized_content);
      setShowOriginal(false);
    } catch (error) {
      console.error('Personalization failed:', error);
      alert('Failed to personalize content. Please try again.');
    } finally {
      setIsPersonalizing(false);
    }
  };

  const handleTranslate = async () => {
    if (!session) {
      alert('Please sign in to translate content');
      return;
    }

    setIsTranslating(true);
    try {
      // Get the chapter content from the current page
      const contentElement = document.querySelector('article');
      const content = contentElement?.innerText || '';

      const result = await apiClient.translateChapter(content, 'Urdu');
      setTranslatedContent(result.translated_content);
      setShowOriginal(false);
    } catch (error) {
      console.error('Translation failed:', error);
      alert('Failed to translate content. Please try again.');
    } finally {
      setIsTranslating(false);
    }
  };

  const handleShowOriginal = () => {
    setShowOriginal(true);
    setPersonalizedContent(null);
    setTranslatedContent(null);
  };

  if (isPending) {
    return null;
  }

  return (
    <div className={styles.chapterTools}>
      <div className={styles.toolsContainer}>
        {session && (
          <>
            <button
              onClick={handlePersonalize}
              disabled={isPersonalizing}
              className={`${styles.toolButton} ${styles.personalizeButton}`}
            >
              {isPersonalizing ? (
                <>
                  <span className={styles.spinner}></span>
                  Personalizing...
                </>
              ) : (
                <>
                  <svg className={styles.icon} fill="none" stroke="currentColor" viewBox="0 0 24 24">
                    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M16 7a4 4 0 11-8 0 4 4 0 018 0zM12 14a7 7 0 00-7 7h14a7 7 0 00-7-7z" />
                  </svg>
                  Personalize for Me
                </>
              )}
            </button>

            <button
              onClick={handleTranslate}
              disabled={isTranslating}
              className={`${styles.toolButton} ${styles.translateButton}`}
            >
              {isTranslating ? (
                <>
                  <span className={styles.spinner}></span>
                  Translating...
                </>
              ) : (
                <>
                  <svg className={styles.icon} fill="none" stroke="currentColor" viewBox="0 0 24 24">
                    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M3 5h12M9 3v2m1.048 9.5A18.022 18.022 0 016.412 9m6.088 9h7M11 21l5-10 5 10M12.751 5C11.783 10.77 8.07 15.61 3 18.129" />
                  </svg>
                  Translate to Urdu
                </>
              )}
            </button>

            {(personalizedContent || translatedContent) && (
              <button
                onClick={handleShowOriginal}
                className={`${styles.toolButton} ${styles.originalButton}`}
              >
                <svg className={styles.icon} fill="none" stroke="currentColor" viewBox="0 0 24 24">
                  <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M3 10h10a8 8 0 018 8v2M3 10l6 6m-6-6l6-6" />
                </svg>
                Show Original
              </button>
            )}
          </>
        )}

        {!session && (
          <div className={styles.signInPrompt}>
            <svg className={styles.icon} fill="none" stroke="currentColor" viewBox="0 0 24 24">
              <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M13 16h-1v-4h-1m1-4h.01M21 12a9 9 0 11-18 0 9 9 0 0118 0z" />
            </svg>
            <span>Sign in to personalize and translate this chapter</span>
          </div>
        )}
      </div>

      {!showOriginal && personalizedContent && (
        <div className={styles.personalizedContent}>
          <div className={styles.contentHeader}>
            <span className={styles.badge}>Personalized for You</span>
          </div>
          <div className={styles.content} dangerouslySetInnerHTML={{ __html: personalizedContent }} />
        </div>
      )}

      {!showOriginal && translatedContent && (
        <div className={styles.translatedContent}>
          <div className={styles.contentHeader}>
            <span className={styles.badge}>اردو میں ترجمہ</span>
          </div>
          <div className={styles.content} dir="rtl" dangerouslySetInnerHTML={{ __html: translatedContent }} />
        </div>
      )}
    </div>
  );
}
