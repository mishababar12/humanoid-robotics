import React, { createContext, useContext, ReactNode, useEffect, useState } from 'react';
import { useSession } from '@site/src/auth/client';

interface UserBackground {
  expertiseLevel?: string;
  background?: string;
  softwareExperience?: string;
  hardwareExperience?: string;
}

interface PersonalizationContextType {
  userBackground: UserBackground | null;
  getPersonalizedPrompt: (basePrompt: string) => string;
  isPersonalized: boolean;
}

const UserPersonalizationContext = createContext<PersonalizationContextType | undefined>(undefined);

export const UserPersonalizationProvider: React.FC<{ children: ReactNode }> = ({ children }) => {
  const sessionData = useSession();
  const session = sessionData?.data;
  const [userBackground, setUserBackground] = useState<UserBackground | null>(null);

  useEffect(() => {
    try {
      if (session && typeof session === 'object' && session.user) {
        const user = session.user;
        const newBackground = {
          expertiseLevel: typeof user.expertiseLevel === 'string' ? user.expertiseLevel : undefined,
          background: typeof user.background === 'string' ? user.background : undefined,
          softwareExperience: typeof user.softwareExperience === 'string' ? user.softwareExperience : undefined,
          hardwareExperience: typeof user.hardwareExperience === 'string' ? user.hardwareExperience : undefined
        };
        setUserBackground(newBackground);
      }
    } catch (error) {
      console.error('Error setting user background:', error instanceof Error ? error.message : String(error));
    }
  }, [session?.user?.expertiseLevel, session?.user?.background, session?.user?.softwareExperience, session?.user?.hardwareExperience]);

  const getPersonalizedPrompt = (basePrompt: string): string => {
    try {
      if (!basePrompt || typeof basePrompt !== 'string') {
        return '';
      }

      console.log('getPersonalizedPrompt called with:', String(basePrompt));
      if (!userBackground || !userBackground.expertiseLevel) {
        console.log('getPersonalizedPrompt returning basePrompt:', String(basePrompt));
        return basePrompt;
      }

      // Add personalization context to the prompt based on user background
      const expertiseContext = userBackground.expertiseLevel
        ? `The user has ${String(userBackground.expertiseLevel)} level expertise.`
        : '';

      const backgroundContext = userBackground.background
        ? `The user comes from a ${String(userBackground.background)} background.`
        : '';

      const experienceContext = userBackground.softwareExperience || userBackground.hardwareExperience
        ? `Relevant experience: Software: ${String(userBackground.softwareExperience || 'N/A')}, Hardware: ${String(userBackground.hardwareExperience || 'N/A')}.`
        : '';

      const personalizationPrefix = [expertiseContext, backgroundContext, experienceContext]
        .filter(context => context)
        .join(' ');

      const result = personalizationPrefix
        ? `${personalizationPrefix} ${basePrompt}`
        : basePrompt;

      console.log('getPersonalizedPrompt returning:', String(result));
      return result;
    } catch (error) {
      console.error('Error in getPersonalizedPrompt:', error);
      return basePrompt;
    }
  };

  const isPersonalized = !!(userBackground &&
    (userBackground.expertiseLevel || userBackground.background ||
     userBackground.softwareExperience || userBackground.hardwareExperience));

  return (
    <UserPersonalizationContext.Provider
      value={{
        userBackground,
        getPersonalizedPrompt,
        isPersonalized
      }}
    >
      {children}
    </UserPersonalizationContext.Provider>
  );
};

export const useUserPersonalization = (): PersonalizationContextType => {
  const context = useContext(UserPersonalizationContext);
  if (context === undefined) {
    throw new Error('useUserPersonalization must be used within a UserPersonalizationProvider');
  }
  return context;
};