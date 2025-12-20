import React from 'react';
import Link from '@docusaurus/Link';
import clsx from 'clsx';
import styles from './HeroSection.module.css';

const HeroSection = ({ title, subtitle, readMorePath }) => {
  return (
    <section className={styles.heroSection}>
      <div className="container">
        <div className={styles.heroContent}>
          <h1 className={styles.bookTitle}>{title || 'Python Data Science Book'}</h1>
          <p className={styles.bookSubtitle}>{subtitle || 'A comprehensive guide to using Python for data science'}</p>
          <div className={styles.heroButtons}>
            <Link
              className={clsx('button button--primary button--lg', styles.readMoreButton)}
              to={readMorePath || '/docs/intro'}
            >
              Read More
            </Link>
          </div>
        </div>
      </div>
    </section>
  );
};

export default HeroSection;