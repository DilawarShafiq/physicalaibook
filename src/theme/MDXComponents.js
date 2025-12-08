import React from 'react';
import ChapterTools from '../components/ChapterTools/ChapterTools';

// Custom MDX components for the textbook
const MDXComponents = {
  // Custom card component
  card: ({ children, ...props }) => (
    <div 
      className="card" 
      style={{
        background: 'linear-gradient(135deg, rgba(26, 115, 232, 0.03), rgba(52, 168, 83, 0.03))',
        border: '1px solid var(--ifm-color-primary-lightest)',
        borderRadius: 'var(--ifm-card-border-radius)',
        padding: '1.5rem',
        margin: '1.5rem 0',
        boxShadow: 'var(--ifm-card-shadow)',
      }}
      {...props}
    >
      {children}
    </div>
  ),

  // Interactive element component
  'interactive-element': ({ children, ...props }) => (
    <div 
      className="interactive-element"
      style={{
        background: 'linear-gradient(135deg, rgba(26, 115, 232, 0.05), rgba(52, 168, 83, 0.05))',
        border: '1px solid var(--ifm-color-primary-lightest)',
        borderRadius: 'var(--ifm-card-border-radius)',
        padding: '1.2rem',
        margin: '1.2rem 0',
        borderLeft: '4px solid var(--ifm-color-primary)',
      }}
      {...props}
    >
      {children}
    </div>
  ),

  // Lab container component
  'lab-container': ({ children, ...props }) => (
    <div 
      className="lab-container"
      {...props}
    >
      {children}
    </div>
  ),

  // Hardware table component
  'hardware-table': ({ children, ...props }) => (
    <div className="table-container" style={{ overflowX: 'auto' }}>
      <table 
        className="hardware-table"
        {...props}
      >
        {children}
      </table>
    </div>
  ),

  // Chapter tools component
  'chapter-tools': ({ chapterId = 'current', title = 'Current Chapter', ...props }) => (
    <ChapterTools 
      chapterId={chapterId} 
      title={title}
      {...props}
    />
  ),

  // Admonition components
  'admonition-note': ({ children, ...props }) => (
    <div 
      className="admonition admonition-note"
      {...props}
    >
      {children}
    </div>
  ),

  'admonition-tip': ({ children, ...props }) => (
    <div 
      className="admonition admonition-tip"
      {...props}
    >
      {children}
    </div>
  ),

  'admonition-caution': ({ children, ...props }) => (
    <div 
      className="admonition admonition-caution"
      {...props}
    >
      {children}
    </div>
  ),
};

export default MDXComponents;