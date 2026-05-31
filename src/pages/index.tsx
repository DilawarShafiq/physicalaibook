import React from 'react';
import Link from '@docusaurus/Link';
import Layout from '@theme/Layout';
import styles from './index.module.css';

type BookCover = {
  vol: string;
  code: string;
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
  author: string;
};

const BOOKS: BookCover[] = [
  {
    vol: '01',
    code: 'PAI',
    badge: 'Robotics',
    title: 'Physical AI & Humanoid Robotics',
    subtitle: 'The Technology Behind Autosapien G1',
    description:
      'Research-grade curriculum covering the 2024–2026 frontier — from QDD actuators to π0 VLA foundation models. Every module informs a design decision in G1.',
    hook:
      'No humanoid has yet demonstrated 8-hour unsupervised household operation. G1 will. This is the curriculum that gets us there.',
    totalHours: '68+',
    modules: 10,
    lessons: 42,
    to: '/physical-ai',
    startLabel: 'Open the book',
    author: 'Dilawar Gopang',
  },
  {
    vol: '02',
    code: 'CEHRS',
    badge: 'Certification Prep',
    title: 'CEHRS Certification Prep',
    subtitle: 'NHA Certified Electronic Health Records Specialist',
    description:
      'Complete preparation for the 130-question NHA CEHRS exam. All six domains, real exam content, and a 10-day countdown study plan.',
    hook:
      '22% of the exam is documentation, 21% medical terminology, 17% HIPAA. You need the exact 130 facts that appear — this book tells you which.',
    totalHours: '44+',
    modules: 10,
    lessons: 36,
    to: '/cehrs',
    startLabel: 'Open the book',
    author: 'Dilawar Gopang',
  },
  {
    vol: '03',
    code: 'AHE',
    badge: 'Agentic AI',
    title: 'AI Healthcare Employees',
    subtitle: 'Building Personal Medical Billers at Scale',
    description:
      'Build, deploy, and scale AI agents that automate US healthcare administration — RCM, prior auth, coding, denial management — on the Autosapien stack.',
    hook:
      'US healthcare spends $500B a year on admin paperwork. One engineer with the right agent framework automates what a team of 20 once did by hand.',
    totalHours: '52+',
    modules: 8,
    lessons: 22,
    to: '/agentic-healthcare',
    startLabel: 'Open the book',
    author: 'Dilawar Gopang',
  },
];

function Hairline() {
  return <span className={styles.hairline} aria-hidden="true" />;
}

function HeroSection() {
  return (
    <header className={styles.hero}>
      <div className={styles.heroGlow} aria-hidden="true" />
      <div className={styles.container}>
        <div className={styles.heroEyebrow}>
          <Hairline />
          <span className={styles.monoLabel}>Autosapien Academy</span>
          <Hairline />
        </div>
        <h1 className={styles.heroTitle}>
          Research-grade books for the
          <br />
          people building physical&nbsp;intelligence.
        </h1>
        <p className={styles.heroSubtitle}>
          Three self-contained curricula — Physical AI &amp; Humanoid Robotics,
          CEHRS Certification Prep, and Agentic Healthcare — written for the
          engineers, researchers, and clinicians shipping real systems.
        </p>
        <div className={styles.heroStats}>
          <div className={styles.heroStat}>
            <span className={styles.heroStatNum}>3</span>
            <span className={styles.heroStatLabel}>Books</span>
          </div>
          <span className={styles.heroDivider} aria-hidden="true" />
          <div className={styles.heroStat}>
            <span className={styles.heroStatNum}>100</span>
            <span className={styles.heroStatLabel}>Lessons</span>
          </div>
          <span className={styles.heroDivider} aria-hidden="true" />
          <div className={styles.heroStat}>
            <span className={styles.heroStatNum}>164+</span>
            <span className={styles.heroStatLabel}>Hours</span>
          </div>
        </div>
      </div>
    </header>
  );
}

function BookCoverCard({ book }: { book: BookCover }) {
  return (
    <article className={styles.cover}>
      <span className={styles.coverSpine} aria-hidden="true" />
      <div className={styles.coverArt}>
        <span className={styles.coverNo} aria-hidden="true">
          {book.vol}
        </span>
        <div className={styles.coverTopRow}>
          <span className={styles.coverBadge}>{book.badge}</span>
          <span className={styles.coverVol}>VOL.&nbsp;{book.vol}</span>
        </div>
        <h3 className={styles.coverTitle}>{book.title}</h3>
        <p className={styles.coverSub}>{book.subtitle}</p>
        <span className={styles.coverWatermark} aria-hidden="true">
          {book.code} · AUTOSAPIEN ACADEMY
        </span>
      </div>

      <div className={styles.coverBody}>
        <p className={styles.coverHook}>“{book.hook}”</p>
        <p className={styles.coverDesc}>{book.description}</p>

        <div className={styles.coverStats}>
          <div className={styles.stat}>
            <span className={styles.statNum}>{book.modules}</span>
            <span className={styles.statLabel}>Modules</span>
          </div>
          <div className={styles.stat}>
            <span className={styles.statNum}>{book.lessons}</span>
            <span className={styles.statLabel}>Lessons</span>
          </div>
          <div className={styles.stat}>
            <span className={styles.statNum}>{book.totalHours}</span>
            <span className={styles.statLabel}>Hours</span>
          </div>
        </div>

        <div className={styles.coverFooter}>
          <span className={styles.coverAuthor}>By {book.author}</span>
          <Link className={styles.coverCta} to={book.to}>
            {book.startLabel}
            <span className={styles.ctaArrow} aria-hidden="true">→</span>
          </Link>
        </div>
      </div>
    </article>
  );
}

function LibrarySection() {
  return (
    <section className={styles.library}>
      <div className={styles.container}>
        <div className={styles.sectionHead}>
          <div className={styles.heroEyebrow}>
            <Hairline />
            <span className={styles.monoLabel}>The Library</span>
            <Hairline />
          </div>
          <h2 className={styles.sectionTitle}>Three research-grade books</h2>
          <p className={styles.sectionSubtitle}>
            Each book is a complete, self-contained curriculum. Start anywhere.
          </p>
        </div>
        <div className={styles.coverGrid}>
          {BOOKS.map((book) => (
            <BookCoverCard key={book.to} book={book} />
          ))}
        </div>
      </div>
    </section>
  );
}

export default function Home(): JSX.Element {
  return (
    <Layout
      title="Home"
      description="Autosapien Academy — three research-grade books on Physical AI & Humanoid Robotics, CEHRS Certification Prep, and Agentic Healthcare.">
      <HeroSection />
      <LibrarySection />
    </Layout>
  );
}
