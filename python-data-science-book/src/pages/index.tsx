import type {ReactNode} from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';

import HeroSection from '../components/Homepage/HeroSection';
import ModulesGrid from '../components/Homepage/ModulesGrid';
import { homepageContent } from '../data/modules-data';

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="A comprehensive guide to Python for Data Science">
      <HeroSection
        title={homepageContent.title}
        subtitle={homepageContent.subtitle}
        readMorePath={homepageContent.readMorePath}
      />
      <ModulesGrid modules={homepageContent.modules} />
    </Layout>
  );
}
