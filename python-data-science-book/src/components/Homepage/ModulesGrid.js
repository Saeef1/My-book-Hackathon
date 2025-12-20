import React from 'react';
import clsx from 'clsx';
import ModuleCard from '../ModuleCard';
import styles from './ModulesGrid.module.css';

const ModulesGrid = ({ modules = [] }) => {
  if (!modules || modules.length === 0) {
    return (
      <section
        className={styles.modulesGridSection}
        aria-labelledby="modules-grid-no-modules"
      >
        <div className="container">
          <h2 id="modules-grid-no-modules" className={styles.sectionTitle}>Book Modules</h2>
          <p className={styles.noModulesMessage}>No modules available at this time.</p>
        </div>
      </section>
    );
  }

  return (
    <section
      className={styles.modulesGridSection}
      aria-labelledby="modules-grid-with-modules"
    >
      <div className="container">
        <h2 id="modules-grid-with-modules" className={styles.sectionTitle}>Book Modules</h2>
        <div
          className={styles.modulesGrid}
          role="list"
          aria-label="List of book modules"
        >
          {modules.map((module) => (
            <ModuleCard
              key={module.id || module.number}
              id={module.id}
              number={module.number}
              title={module.title}
              description={module.description}
              path={module.path}
              imageUrl={module.imageUrl}
            />
          ))}
        </div>
      </div>
    </section>
  );
};

export default ModulesGrid;