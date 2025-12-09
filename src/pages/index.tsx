import React from 'react';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import styles from './index.module.css';

function HeroSection() {
  return (
    <header className={styles.heroBanner}>
      <div className={styles.container}>
        <div className={styles.heroContent}>
          <div className={styles.heroText}>
            <h1 className={styles.heroTitle}>
              Physical AI & <br />
              <span className={styles.gradient}>Humanoid Robotics</span>
            </h1>
            <p className={styles.heroSubtitle}>
              Master the convergence of AI, robotics, and embodied intelligence.
              Build intelligent systems that perceive, reason, and act in the physical world.
            </p>
            <div className={styles.buttons}>
              <Link
                className={styles.primaryButton}
                to="/curriculum/introduction">
                Start Learning →
              </Link>
              <Link
                className={styles.secondaryButton}
                to="/curriculum/schedule">
                View Schedule
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

function FeatureCard({ icon, title, description }) {
  return (
    <div className={styles.featureCard}>
      <div className={styles.featureIcon}>{icon}</div>
      <h3 className={styles.featureTitle}>{title}</h3>
      <p className={styles.featureDescription}>{description}</p>
    </div>
  );
}

function FeaturesSection() {
  const features = [
    {
      icon: '🤖',
      title: 'ROS 2 Mastery',
      description: 'Learn the industry-standard framework for building modular, distributed robotics systems with hands-on projects.'
    },
    {
      icon: '🎮',
      title: 'Digital Twin Simulation',
      description: 'Master Gazebo and NVIDIA Isaac Sim for high-fidelity robot simulation and sim-to-real transfer.'
    },
    {
      icon: '👁️',
      title: 'AI Perception',
      description: 'Implement computer vision, sensor fusion, and real-time perception with state-of-the-art models.'
    },
    {
      icon: '🧠',
      title: 'LLM Integration',
      description: 'Connect GPT-4, Claude, and Gemini to robotic systems for natural language command interfaces.'
    },
    {
      icon: '🎯',
      title: 'VLA Models',
      description: 'Deploy Vision-Language-Action models like RT-2 and OpenVLA for end-to-end robot control.'
    },
    {
      icon: '⚡',
      title: 'Edge Deployment',
      description: 'Optimize and deploy AI models to NVIDIA Jetson edge devices for real-time robotics.'
    }
  ];

  return (
    <section className={styles.featuresSection}>
      <div className={styles.container}>
        <h2 className={styles.sectionTitle}>What You'll Learn</h2>
        <div className={styles.featuresGrid}>
          {features.map((feature, idx) => (
            <FeatureCard key={idx} {...feature} />
          ))}
        </div>
      </div>
    </section>
  );
}

function ModulesSection() {
  const modules = [
    {
      number: '01',
      title: 'ROS 2 Fundamentals',
      weeks: 'Weeks 1-3',
      topics: ['Nodes & Topics', 'Services & Actions', 'Navigation2', 'Transform (tf2)'],
      color: '#6366f1'
    },
    {
      number: '02',
      title: 'Digital Twin & Gazebo',
      weeks: 'Weeks 4-6',
      topics: ['Gazebo Simulation', 'Sim-to-Real Transfer', 'Multi-Robot Systems', 'Domain Randomization'],
      color: '#06b6d4'
    },
    {
      number: '03',
      title: 'AI-Robot Brain & Isaac',
      weeks: 'Weeks 7-10',
      topics: ['Computer Vision', 'NVIDIA Isaac Sim', 'Reinforcement Learning', 'Perception Pipelines'],
      color: '#a855f7'
    },
    {
      number: '04',
      title: 'VLA & LLMs',
      weeks: 'Weeks 11-13',
      topics: ['LLM Integration', 'VLA Models (RT-2)', 'Voice Control', 'Capstone Project'],
      color: '#f97316'
    }
  ];

  return (
    <section className={styles.modulesSection}>
      <div className={styles.container}>
        <h2 className={styles.sectionTitle}>Course Modules</h2>
        <p className={styles.sectionSubtitle}>
          A comprehensive 13-week journey from fundamentals to cutting-edge Physical AI
        </p>
        <div className={styles.modulesGrid}>
          {modules.map((module, idx) => (
            <div key={idx} className={styles.moduleCard} style={{ borderColor: module.color }}>
              <div className={styles.moduleNumber} style={{ color: module.color }}>
                {module.number}
              </div>
              <h3 className={styles.moduleTitle}>{module.title}</h3>
              <p className={styles.moduleWeeks}>{module.weeks}</p>
              <ul className={styles.moduleTopics}>
                {module.topics.map((topic, i) => (
                  <li key={i}>{topic}</li>
                ))}
              </ul>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

function StatsSection() {
  const stats = [
    { number: '13', label: 'Weeks of Content' },
    { number: '4', label: 'Complete Modules' },
    { number: '30+', label: 'Hands-On Labs' },
    { number: '100%', label: 'Project-Based' }
  ];

  return (
    <section className={styles.statsSection}>
      <div className={styles.container}>
        <div className={styles.statsGrid}>
          {stats.map((stat, idx) => (
            <div key={idx} className={styles.statCard}>
              <div className={styles.statNumber}>{stat.number}</div>
              <div className={styles.statLabel}>{stat.label}</div>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

function CTASection() {
  return (
    <section className={styles.ctaSection}>
      <div className={styles.container}>
        <div className={styles.ctaContent}>
          <h2 className={styles.ctaTitle}>Ready to Build Intelligent Robots?</h2>
          <p className={styles.ctaSubtitle}>
            Start your journey into Physical AI and humanoid robotics today.
          </p>
          <Link
            className={styles.ctaButton}
            to="/curriculum/introduction">
            Begin Course →
          </Link>
        </div>
      </div>
    </section>
  );
}

export default function Home(): JSX.Element {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title="Home"
      description="Master Physical AI & Humanoid Robotics - A comprehensive course covering ROS 2, simulation, AI perception, LLMs, and VLA models">
      <HeroSection />
      <FeaturesSection />
      <StatsSection />
      <ModulesSection />
      <CTASection />
    </Layout>
  );
}
