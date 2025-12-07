import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import Heading from '@theme/Heading';
import HeroImage from '@site/static/img/hero-robot-placeholder.svg'; // New import
import RobotHeroImage from '@site/static/img/robot-hero.jpeg'; // New import

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
              <div className={clsx('container', styles.heroContainer)}>
                <div className={styles.heroText}>
                  <Heading as="h1" className={styles.heroTitle}>
                    {siteConfig.title}
                            </Heading>                  <div className={styles.buttons}>
                    <Link
                      className="button button--secondary button--lg"
                      to="/docs/intro">
                      Start Learning ⏱️
                    </Link>
                  </div>
                </div>
                <div className={styles.heroImage}>
                  <img src={RobotHeroImage} alt="Physical AI Robot" /> {/* Updated src */}
                </div>
              </div>
    </header>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Hello from ${siteConfig.title}`}
      description="Description will go into a meta tag in <head />">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
      </main>
    </Layout>
  );
}
