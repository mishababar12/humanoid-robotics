import React, { useEffect, useState } from 'react';
import Layout from '@theme-original/Layout';
import type { Props } from '@theme/Layout';
import { UserPersonalizationProvider } from '@site/src/contexts/UserPersonalizationContext';
import ErrorBoundary from '@site/src/components/ErrorBoundary/ErrorBoundary';

// Create a client-only component for the auth provider
const ClientAuthProvider: React.FC<{ children: React.ReactNode }> = ({ children }) => {
  const [AuthClientProvider, setAuthClientProvider] = useState<React.ComponentType<{ children: React.ReactNode }> | null>(null);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    // Dynamically import the auth client only on the client side
    import('@site/src/auth/client')
      .then(({ authClient }) => {
        if (authClient && authClient.Provider) {
          setAuthClientProvider(() => authClient.Provider);
        } else {
          setError('Auth client Provider not found');
        }
      })
      .catch((err) => {
        const errorMessage = err instanceof Error ? err.message : String(err);
        console.error('Failed to load auth client:', errorMessage);
        setError(errorMessage);
      });
  }, []);

  // Show error as string, not object
  if (error) {
    console.warn('Auth provider error:', error);
    return <>{children}</>;
  }

  if (AuthClientProvider) {
    return <AuthClientProvider>{children}</AuthClientProvider>;
  }

  // Render children without auth provider during initial render
  return <>{children}</>;
};

// Lazy load components that use auth to avoid SSR issues
const LazyRagChatbot: React.FC = () => {
  const [RagChatbotComponent, setRagChatbotComponent] = useState<React.ComponentType | null>(null);
  const [isClient, setIsClient] = useState(false);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    setIsClient(true);
    if (typeof window !== 'undefined') {
      import('@site/src/components/RagChatbot/RagChatbot')
        .then((module) => {
          setRagChatbotComponent(() => module.default);
        })
        .catch((err) => {
          const errorMessage = err instanceof Error ? err.message : String(err);
          console.error('Failed to load RagChatbot:', errorMessage);
          setError(errorMessage);
        });
    }
  }, []);

  if (error) {
    return null; // Silently fail
  }

  if (isClient && RagChatbotComponent) {
    return <RagChatbotComponent />;
  }

  return null;
};

const LazyUserBackgroundQuestionnaire: React.FC = () => {
  const [QuestionnaireComponent, setQuestionnaireComponent] = useState<React.ComponentType | null>(null);
  const [isClient, setIsClient] = useState(false);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    setIsClient(true);
    if (typeof window !== 'undefined') {
      import('@site/src/components/UserBackgroundQuestionnaire/UserBackgroundQuestionnaire')
        .then((module) => {
          setQuestionnaireComponent(() => module.default);
        })
        .catch((err) => {
          const errorMessage = err instanceof Error ? err.message : String(err);
          console.error('Failed to load UserBackgroundQuestionnaire:', errorMessage);
          setError(errorMessage);
        });
    }
  }, []);

  if (error) {
    return null; // Silently fail
  }

  if (isClient && QuestionnaireComponent) {
    return <QuestionnaireComponent />;
  }

  return null;
};

export default function LayoutWrapper(props: Props): React.ReactElement {
  const [isClient, setIsClient] = useState(false);

  useEffect(() => {
    setIsClient(true);
  }, []);

  // Ensure props are safe to pass
  const safeProps = {
    ...props,
    title: typeof props.title === 'string' ? props.title : String(props.title || ''),
    description: typeof props.description === 'string' ? props.description : String(props.description || '')
  };

  // During SSR, render without client-side features
  if (!isClient) {
    return (
      <Layout {...safeProps}>
        {props.children}
      </Layout>
    );
  }

  // Client-side: Enable chatbot only (without auth for now)
  return (
    <Layout {...safeProps}>
      {props.children}
      <ErrorBoundary fallback={null}>
        <LazyRagChatbot />
      </ErrorBoundary>
    </Layout>
  );
}
