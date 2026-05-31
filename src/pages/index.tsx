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
    totalHours: '73+',
    modules: 11,
    lessons: 45,
    to: '/physical-ai',
    startLabel: 'Open the book',
    author: 'Dilawar Gopang',
  },
  {
    vol: '02',
    code: 'CEHRS',
    badge: 'Certification Prep',
    title: 'Certified Electronic Health Records Specialist',
    subtitle: 'NHA CEHRS Certification — Complete Exam Prep',
    description:
      'Complete preparation for the 125-question NHA CEHRS exam. All six domains, real exam content, and a 10-day countdown study plan.',
    hook:
      '22% of the exam is documentation, 21% medical terminology, 17% HIPAA. You need the exact facts that appear — this book tells you which.',
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
    totalHours: '57+',
    modules: 9,
    lessons: 25,
    to: '/agentic-healthcare',
    startLabel: 'Open the book',
    author: 'Dilawar Gopang',
  },
];

const HOW_IT_WORKS = [
  {
    k: '01',
    title: 'Learn the frontier',
    body: 'Every lesson is grounded in real papers, platforms, and benchmarks from the 2024–2026 state of the art — not generic overviews.',
  },
  {
    k: '02',
    title: 'Practice with labs',
    body: 'Each lesson ends with a hands-on lab: a concrete exercise that turns the concept into something you have actually built or measured.',
  },
  {
    k: '03',
    title: 'Ask the AI tutor',
    body: 'Highlight any passage in a book and ask a question. The Academy tutor answers from the book itself, with sources — like a teacher on call.',
  },
];

const PILLARS = [
  {
    title: 'Grounded in primary sources',
    body: 'Claims trace to cited papers and real systems. You learn what is actually true at the frontier, with the references to verify it.',
  },
  {
    title: 'Built toward real products',
    body: 'The curriculum maps directly onto what Autosapien ships — the G1 humanoid, xEHR, and agentic healthcare. Theory with a destination.',
  },
  {
    title: 'Hands-on by default',
    body: 'Every lesson carries a lab. You leave each one with an artifact, a measurement, or a working prototype — not just notes.',
  },
  {
    title: 'An AI tutor that knows the book',
    body: 'Retrieval-grounded answers from the exact lessons you are reading. Highlight, ask, and get a sourced explanation in context.',
  },
  {
    title: 'Active recall built in',
    body: 'Each lesson includes “check your understanding” cards so you test recall as you go, not just at the end.',
  },
  {
    title: 'Free and self-paced',
    body: 'Start at any book, any module. Prerequisites and durations are marked so you can chart the path that fits your goals.',
  },
];

const PATH = [
  {
    tier: 'Foundational',
    title: 'Build the base',
    body: 'Landscape, first principles, and vocabulary. Start here if the field is new — these modules assume nothing.',
  },
  {
    tier: 'Intermediate',
    title: 'Go deep on systems',
    body: 'Architectures, models, and trade-offs. Where the real engineering decisions live and how to reason about them.',
  },
  {
    tier: 'Advanced',
    title: 'Reach the frontier',
    body: 'Open problems, roadmaps, and the decisions that define what gets built next. The edge of what is known.',
  },
];

const STUDY = [
  { n: '1', t: 'Pick a path', d: 'Choose a book and start at module one, or jump to the topic you need. Durations are marked on every lesson.' },
  { n: '2', t: 'Read, then build', d: 'Work each lesson’s lab before moving on. The labs are where the learning sticks.' },
  { n: '3', t: 'Recall, don’t reread', d: 'Use the “check your understanding” cards to test yourself. Revisit what you miss.' },
  { n: '4', t: 'Ask when stuck', d: 'Highlight the passage and ask the AI tutor. It answers from the book, with sources.' },
];

function Hairline() {
  return <span className={styles.hairline} aria-hidden="true" />;
}

function Eyebrow({ children }: { children: React.ReactNode }) {
  return (
    <div className={styles.eyebrow}>
      <Hairline />
      <span className={styles.monoLabel}>{children}</span>
      <Hairline />
    </div>
  );
}

function HeroSection() {
  return (
    <header className={styles.hero}>
      <div className={styles.heroGlow} aria-hidden="true" />
      <div className={styles.container}>
        <Eyebrow>Autosapien Academy</Eyebrow>
        <h1 className={styles.heroTitle}>
          Research-grade books for the people
          <br />
          building physical&nbsp;intelligence.
        </h1>
        <p className={styles.heroSubtitle}>
          Three self-contained curricula — Physical AI &amp; Humanoid Robotics,
          CEHRS Certification Prep, and Agentic Healthcare — written for the
          engineers, researchers, and clinicians shipping real systems.
        </p>
        <div className={styles.heroButtons}>
          <Link className={styles.primaryBtn} to="/physical-ai">
            Start reading <span aria-hidden="true">→</span>
          </Link>
          <a className={styles.ghostBtn} href="#library">
            Browse the library
          </a>
        </div>
        <div className={styles.heroStats}>
          <div className={styles.heroStat}>
            <span className={styles.heroStatNum}>3</span>
            <span className={styles.heroStatLabel}>Books</span>
          </div>
          <span className={styles.heroDivider} aria-hidden="true" />
          <div className={styles.heroStat}>
            <span className={styles.heroStatNum}>106</span>
            <span className={styles.heroStatLabel}>Lessons</span>
          </div>
          <span className={styles.heroDivider} aria-hidden="true" />
          <div className={styles.heroStat}>
            <span className={styles.heroStatNum}>174+</span>
            <span className={styles.heroStatLabel}>Hours</span>
          </div>
        </div>
      </div>
    </header>
  );
}

function HowItWorks() {
  return (
    <section className={styles.band}>
      <div className={styles.container}>
        <Eyebrow>How the Academy works</Eyebrow>
        <h2 className={styles.sectionTitle}>Learn it, build it, ask about it</h2>
        <div className={styles.tripleGrid}>
          {HOW_IT_WORKS.map((c) => (
            <div key={c.k} className={styles.featureCard}>
              <span className={styles.featureKey}>{c.k}</span>
              <h3 className={styles.featureTitle}>{c.title}</h3>
              <p className={styles.featureBody}>{c.body}</p>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

function BookCoverCard({ book }: { book: BookCover }) {
  return (
    <article className={styles.cover}>
      <span className={styles.coverSpine} aria-hidden="true" />
      <div className={styles.coverArt}>
        <span className={styles.coverNo} aria-hidden="true">{book.vol}</span>
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
    <section id="library" className={styles.bandAlt}>
      <div className={styles.container}>
        <Eyebrow>The Library</Eyebrow>
        <h2 className={styles.sectionTitle}>Three research-grade books</h2>
        <p className={styles.sectionSubtitle}>
          Each book is a complete, self-contained curriculum. Start anywhere.
        </p>
        <div className={styles.coverGrid}>
          {BOOKS.map((book) => (
            <BookCoverCard key={book.to} book={book} />
          ))}
        </div>
      </div>
    </section>
  );
}

function PillarsSection() {
  return (
    <section className={styles.band}>
      <div className={styles.container}>
        <Eyebrow>Why it’s different</Eyebrow>
        <h2 className={styles.sectionTitle}>Built like an engineering book, not a blog</h2>
        <div className={styles.pillarGrid}>
          {PILLARS.map((p) => (
            <div key={p.title} className={styles.pillar}>
              <span className={styles.pillarMark} aria-hidden="true" />
              <h3 className={styles.pillarTitle}>{p.title}</h3>
              <p className={styles.pillarBody}>{p.body}</p>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

function PathwaySection() {
  return (
    <section className={styles.bandAlt}>
      <div className={styles.container}>
        <Eyebrow>Learning pathway</Eyebrow>
        <h2 className={styles.sectionTitle}>From first principles to the frontier</h2>
        <p className={styles.sectionSubtitle}>
          Every module is tagged by level, so you can climb at your own pace.
        </p>
        <div className={styles.pathGrid}>
          {PATH.map((p, i) => (
            <div key={p.tier} className={styles.pathStep}>
              <span className={styles.pathTier}>{p.tier}</span>
              <h3 className={styles.pathTitle}>{p.title}</h3>
              <p className={styles.pathBody}>{p.body}</p>
              {i < PATH.length - 1 && (
                <span className={styles.pathArrow} aria-hidden="true">↓</span>
              )}
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

function StudySection() {
  return (
    <section className={styles.band}>
      <div className={styles.container}>
        <Eyebrow>How to study</Eyebrow>
        <h2 className={styles.sectionTitle}>A simple loop that compounds</h2>
        <div className={styles.studyGrid}>
          {STUDY.map((s) => (
            <div key={s.n} className={styles.studyStep}>
              <span className={styles.studyNum}>{s.n}</span>
              <div>
                <h3 className={styles.studyTitle}>{s.t}</h3>
                <p className={styles.studyBody}>{s.d}</p>
              </div>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

function CtaSection() {
  return (
    <section className={styles.cta}>
      <div className={styles.ctaGlow} aria-hidden="true" />
      <div className={styles.container}>
        <h2 className={styles.ctaTitle}>Start with the Humanoid Landscape</h2>
        <p className={styles.ctaSubtitle}>
          Ten modules, forty-two lessons, sixty-eight hours — the curriculum
          behind Autosapien G1. Free, and ready when you are.
        </p>
        <Link className={styles.primaryBtn} to="/physical-ai">
          Open Physical AI &amp; Humanoid Robotics <span aria-hidden="true">→</span>
        </Link>
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
      <HowItWorks />
      <LibrarySection />
      <PillarsSection />
      <PathwaySection />
      <StudySection />
      <CtaSection />
    </Layout>
  );
}
