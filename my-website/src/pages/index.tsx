import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import useBaseUrl from '@docusaurus/useBaseUrl';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import Heading from '@theme/Heading';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  const robotImageUrl = useBaseUrl('/img/animated-robot.svg');
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
              <div className={clsx('container', styles.heroContainer)}>
                <div className={styles.heroText}>
                  <div className={styles.heroContent}>
                    <Heading as="h1" className={styles.heroTitle}>
                      Physical AI & Humanoid Robotics
                    </Heading>
                    <p className={styles.heroSubtitle}>
                      Master the future of intelligent machines. Learn ROS2, motion planning, computer vision, and LLM-powered robotics to build real-world AI systems that think, move, and interact.
                    </p>
                    <div className={styles.buttons}>
                      <Link
                        className="button button--secondary button--lg"
                        to="/docs/intro">
                        Start Learning ⏱️
                      </Link>
                    </div>
                  </div>
                </div>
                <div className={styles.heroImage}>
                  <img
                    src={robotImageUrl}
                    alt="Animated AI Robot"
                    className={styles.heroRobotImage}
                  />
                </div>
              </div>
    </header>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Hello from ${String(siteConfig?.title || 'Physical AI')}`}
      description="Description will go into a meta tag in <head />">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
      </main>
    </Layout>
  );
}
