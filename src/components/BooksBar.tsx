import React from 'react';
import Link from '@docusaurus/Link';
import {useLocation} from '@docusaurus/router';
import styles from './BooksBar.module.css';

const BOOKS = [
  {short: 'Physical AI', to: '/physical-ai'},
  {short: 'CEHRS', to: '/cehrs'},
  {short: 'Agentic Healthcare', to: '/agentic-healthcare'},
];

export default function BooksBar(): JSX.Element {
  const {pathname} = useLocation();
  return (
    <nav className={styles.booksBar} aria-label="Books">
      <div className={styles.inner}>
        <span className={styles.eyebrow}>Books</span>
        <div className={styles.links}>
          {BOOKS.map((b) => {
            const active = pathname === b.to || pathname.startsWith(b.to + '/');
            return (
              <Link
                key={b.to}
                to={b.to}
                className={active ? styles.active : styles.link}>
                {b.short}
              </Link>
            );
          })}
        </div>
      </div>
    </nav>
  );
}
