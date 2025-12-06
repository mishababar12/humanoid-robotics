import type {ReactNode} from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  description: ReactNode;
};

// Feature list without SVGs
const FeatureList: FeatureItem[] = [
  {
    title: 'Hands-on Robotics',
    description: (
      <>Learn by building real-world humanoid robots and integrating AI for autonomous control.</>
    ),
  },
  {
    title: 'AI-Powered Control',
    description: (
      <>Explore how AI and machine learning can be applied to robotics and digital twins.</>
    ),
  },
  {
    title: 'Comprehensive Learning',
    description: (
      <>Cover everything from ROS2 foundations, sensors, and vision systems to full humanoid AI integration.</>
    ),
  },
];

function Feature({title, description}: FeatureItem) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center mb-4">
        {/* Optional: you can add an emoji/icon here instead of SVG */}
        <span style={{fontSize: '3rem'}}>🤖</span>
      </div>
      <div className="text--center padding-horiz--md">
        <Heading as="h3" style={{color: '#7a3fc0'}}>
          {title}
        </Heading>
        <p style={{color: '#d3a0ff'}}>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): ReactNode {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
