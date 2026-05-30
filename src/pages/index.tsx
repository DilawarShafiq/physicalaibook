import React from 'react';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import styles from './index.module.css';

type BookCard = {
  eyebrow: string;
  badge: string;
  title: string;
  subtitle: string;
  description: string;
  hook: string;
  totalHours: string;
  modules: number;
  lessons: number;
  to: string;
  startLabel: string;
  color: string;
};

const BOOKS: BookCard[] = [
  {
    eyebrow: 'Book 1',
    badge: 'ROBOTICS',
    title: 'Physical AI & Humanoid Robotics',
    subtitle: 'The Technology Behind Autosapien G1',
    description:
      'Research-grade curriculum covering the 2024–2026 frontier. From QDD actuators to π0 VLA foundation models — every module informs a design decision in G1.',
    hook:
      'No humanoid has yet demonstrated 8-hour unsupervised household operation. G1 will. This is the curriculum that gets us there.',
    totalHours: '68+',
    modules: 10,
    lessons: 42,
    to: '/physical-ai',
    startLabel: 'Start with the Humanoid Landscape',
    color: '#6366f1',
  },
  {
    eyebrow: 'Book 2',
    badge: 'CERTIFICATION PREP',
    title: 'CEHRS Certification Prep',
    subtitle: 'NHA Certified Electronic Health Records Specialist',
    description:
      'Complete preparation for the 130-question NHA CEHRS exam. All 6 domains, real exam content, and a 10-day countdown study plan.',
    hook:
      '22% of the exam is documentation. 21% is medical terminology. 17% is HIPAA. You need to know exactly which 130 facts appear — this book tells you.',
    totalHours: '44+',
    modules: 10,
    lessons: 36,
    to: '/cehrs',
    startLabel: 'Start with the Exam Blueprint',
    color: '#06b6d4',
  },
  {
    eyebrow: 'Book 3',
    badge: 'AGENTIC AI',
    title: 'AI Healthcare Employees',
    subtitle: 'Building Personal Medical Billers at Scale',
    description:
      'Build, deploy, and scale AI agents that automate US healthcare administration. RCM, prior auth, medical coding, denial management, and the complete Personal Medical Biller — on the Autosapien stack.',
    hook:
      'US healthcare spends $500B/year on admin paperwork. One engineer with the right agent framework can automate what a team of 20 used to do manually.',
    totalHours: '52+',
    modules: 8,
    lessons: 22,
    to: '/agentic-healthcare',
    startLabel: 'Start with the $500B Problem',
    color: '#a855f7',
  },
];

function HeroSection() {
  return (
    <header className={styles.heroBanner}>
      <div className={styles.container}>
        <div className={styles.heroContent}>
          <div className={styles.heroText}>
            <h1 className={styles.heroTitle}>
              Autosapien <br />
              <span className={styles.gradient}>Academy</span>
            </h1>
            <p className={styles.heroSubtitle}>
              Three research-grade books — Physical AI &amp; Humanoid Robotics,
              CEHRS Certification Prep, and Agentic Healthcare. Built for
              engineers, researchers, and healthcare technologists.
            </p>
            <div className={styles.buttons}>
              <Link className={styles.primaryButton} to="/physical-ai">
                Explore the Books →
              </Link>
            </div>
          </div>
          <div className={styles.heroImage}>
            <div className={styles.robotIllustration}>
              <div className={styles.circuit}></div>
              <div className={styles.circuit2}></div>
              <div className={styles.glow}></div>
            </div>
          </div>
        </div>
      </div>
    </header>
  );
}

function BooksSection() {
  return (
    <section className={styles.modulesSection}>
      <div className={styles.container}>
        <h2 className={styles.sectionTitle}>The Three Books</h2>
        <p className={styles.sectionSubtitle}>
          Each book is a complete, self-contained curriculum. Start anywhere.
        </p>
        <div className={styles.modulesGrid}>
          {BOOKS.map((book) => (
            <div
              key={book.to}
              className={styles.moduleCard}
              style={{ borderColor: book.color }}>
              <div className={styles.moduleNumber} style={{ color: book.color }}>
                {book.eyebrow}
              </div>
              <h3 className={styles.moduleTitle}>{book.title}</h3>
              <p className={styles.moduleWeeks}>{book.subtitle}</p>
              <p
                style={{
                  fontStyle: 'italic',
                  color: 'var(--ifm-color-emphasis-700)',
                  marginBottom: '0.75rem',
                }}>
                “{book.hook}”
              </p>
              <p>{book.description}</p>
              <p style={{ fontSize: '0.85rem', opacity: 0.8 }}>
                {book.modules} modules · {book.lessons} lessons ·{' '}
                {book.totalHours} hours
              </p>
              <Link
                className={styles.primaryButton}
                style={{ marginTop: '0.5rem', display: 'inline-block' }}
                to={book.to}>
                {book.startLabel} →
              </Link>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

export default function Home(): JSX.Element {
  const { siteConfig } = useDocusaurusContext();
  return (
    <Layout
      title="Home"
      description="Autosapien Academy — three research-grade books on Physical AI & Humanoid Robotics, CEHRS Certification Prep, and Agentic Healthcare.">
      <HeroSection />
      <BooksSection />
    </Layout>
  );
}
