import React from 'react';
import Link from '@docusaurus/Link';
import clsx from 'clsx';
import styles from './ModuleCard.module.css'; // Using CSS module for encapsulation

const ModuleCard = ({
  id,
  number,
  title,
  description,
  path,
  imageUrl
}) => {
  return (
    <Link
      to={path}
      className={styles.moduleCardLink}
      aria-label={`Module ${number}: ${title}. ${description}`}
    >
      <div
        className={styles.moduleCard}
        role="listitem"
        tabIndex="0"
      >
        {imageUrl && (
          <div className={styles.moduleCardImage}>
            <img
              src={imageUrl}
              alt={`${title} module visual`}
              loading="lazy"
            />
          </div>
        )}
        <div className={styles.moduleCardContent}>
          <div className={styles.moduleNumber} aria-label={`Module number ${number}`}>
            Module {number}
          </div>
          <h3 className={styles.moduleTitle}>{title}</h3>
          <p className={styles.moduleDescription}>{description}</p>
        </div>
      </div>
    </Link>
  );
};

export default ModuleCard;