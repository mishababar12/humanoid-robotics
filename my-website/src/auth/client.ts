import { createAuthClient } from "better-auth/react";

const getBaseUrl = (): string => {
  if (typeof window !== 'undefined') {
    try {
      return window.location.origin;
    } catch (error) {
      console.error('Error getting window.location.origin:', error);
      return 'http://localhost:3000';
    }
  }
  return 'http://localhost:3000';
};

export const {
  signIn,
  signOut,
  useSession,
  useAuth,
  authClient
} = createAuthClient({
  fetchOptions: {
    baseUrl: getBaseUrl(),
  },
});