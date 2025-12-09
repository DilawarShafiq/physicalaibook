/**
 * Better Auth Client
 * Provides client-side authentication utilities for Docusaurus
 */

import { useState, useEffect, createContext, useContext } from 'react';

// Define types for user and session
interface User {
  id: string;
  email: string;
  name: string;
  image?: string;
  // Student profile fields
  softwareExperience?: string;
  hardwareExperience?: string;
  enrollmentDate?: string;
  completedModules?: string[]; // JSON string converted to array
  completedLabs?: string[]; // JSON string converted to array
  currentModule?: number;
  progress?: number;
  preferredLanguage?: string;
  usePersonalization?: boolean;
  fullName?: string;
  institution?: string;
  role?: string;
}

interface Session {
  user: User;
  expires: string;
}

interface AuthState {
  user: User | null;
  session: Session | null;
  loading: boolean;
  error: string | null;
}

// Create auth context
const AuthContext = createContext<{
  state: AuthState;
  signIn: (email: string, password: string) => Promise<void>;
  signUp: (email: string, password: string, name: string) => Promise<void>;
  signOut: () => Promise<void>;
  updateProfile: (profile: Partial<User>) => Promise<void>;
  updateProgress: (moduleId: number, progress: number) => Promise<void>;
  completeModule: (moduleId: number) => Promise<void>;
  completeLab: (labId: string) => Promise<void>;
} | null>(null);

// Auth provider component
export const AuthProvider: React.FC<{ children: React.ReactNode }> = ({ children }) => {
  const [state, setState] = useState<AuthState>({
    user: null,
    session: null,
    loading: true,
    error: null,
  });

  // Get auth state on component mount
  useEffect(() => {
    const getAuthState = async () => {
      try {
        const response = await fetch('/api/auth/get-session', {
          credentials: 'include',
        });
        if (response.ok) {
          const session = await response.json();
          setState({
            user: session?.user || null,
            session: session || null,
            loading: false,
            error: null,
          });
        } else {
          setState({
            user: null,
            session: null,
            loading: false,
            error: null,
          });
        }
      } catch (error) {
        console.error('Error getting auth state:', error);
        setState({
          user: null,
          session: null,
          loading: false,
          error: error instanceof Error ? error.message : 'An error occurred',
        });
      }
    };

    getAuthState();
  }, []);

  const signIn = async (email: string, password: string) => {
    try {
      setState(prev => ({ ...prev, loading: true, error: null }));
      
      const response = await fetch('/api/auth/sign-in/email', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ email, password }),
        credentials: 'include',
      });

      if (!response.ok) {
        const error = await response.json();
        throw new Error(error.message || 'Sign in failed');
      }

      const session = await response.json();
      setState({
        user: session.user,
        session,
        loading: false,
        error: null,
      });
    } catch (error) {
      setState(prev => ({
        ...prev,
        loading: false,
        error: error instanceof Error ? error.message : 'Sign in failed',
      }));
      throw error;
    }
  };

  const signUp = async (email: string, password: string, name: string) => {
    try {
      setState(prev => ({ ...prev, loading: true, error: null }));
      
      const response = await fetch('/api/auth/sign-up/email', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ 
          email, 
          password, 
          name,
          // Include student profile data
          softwareExperience: 'beginner',
          hardwareExperience: 'none',
          preferredLanguage: 'en',
          role: 'student'
        }),
        credentials: 'include',
      });

      if (!response.ok) {
        const error = await response.json();
        throw new Error(error.message || 'Sign up failed');
      }

      const session = await response.json();
      setState({
        user: session.user,
        session,
        loading: false,
        error: null,
      });
    } catch (error) {
      setState(prev => ({
        ...prev,
        loading: false,
        error: error instanceof Error ? error.message : 'Sign up failed',
      }));
      throw error;
    }
  };

  const signOut = async () => {
    try {
      setState(prev => ({ ...prev, loading: true }));
      
      await fetch('/api/auth/sign-out', {
        method: 'POST',
        credentials: 'include',
      });

      setState({
        user: null,
        session: null,
        loading: false,
        error: null,
      });
    } catch (error) {
      setState(prev => ({
        ...prev,
        loading: false,
        error: error instanceof Error ? error.message : 'Sign out failed',
      }));
    }
  };

  const updateProfile = async (profile: Partial<User>) => {
    try {
      if (!state.user) throw new Error('Not authenticated');

      const response = await fetch('/api/auth/user', {
        method: 'POST', // Better Auth uses POST for updates
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(profile),
        credentials: 'include',
      });

      if (!response.ok) {
        throw new Error('Failed to update profile');
      }

      const updatedUser = await response.json();
      setState(prev => ({
        ...prev,
        user: { ...prev.user, ...updatedUser },
        session: prev.session ? { ...prev.session, user: { ...prev.session.user, ...updatedUser } } : prev.session
      }));
    } catch (error) {
      setState(prev => ({
        ...prev,
        error: error instanceof Error ? error.message : 'Update profile failed',
      }));
    }
  };

  // Custom API calls for student progress
  const updateProgress = async (moduleId: number, progress: number) => {
    try {
      if (!state.session) throw new Error('Not authenticated');

      const response = await fetch('/api/auth/update-progress', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ moduleId, progress }),
        credentials: 'include', // Include session cookies
      });

      if (!response.ok) {
        throw new Error('Failed to update progress');
      }

      // Update local state
      const updatedUser = { currentModule: moduleId, progress };
      setState(prev => ({
        ...prev,
        user: prev.user ? { ...prev.user, ...updatedUser } : null,
        session: prev.session ? { ...prev.session, user: { ...prev.session.user, ...updatedUser } } : null
      }));
    } catch (error) {
      setState(prev => ({
        ...prev,
        error: error instanceof Error ? error.message : 'Update progress failed',
      }));
    }
  };

  const completeModule = async (moduleId: number) => {
    try {
      if (!state.session) throw new Error('Not authenticated');

      const response = await fetch('/api/auth/complete-module', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ moduleId }),
        credentials: 'include', // Include session cookies
      });

      if (!response.ok) {
        throw new Error('Failed to complete module');
      }

      const result = await response.json();

      // Update local state
      const updatedUser = { completedModules: result.completedModules };
      setState(prev => ({
        ...prev,
        user: prev.user ? { ...prev.user, ...updatedUser } : null,
        session: prev.session ? { ...prev.session, user: { ...prev.session.user, ...updatedUser } } : null
      }));
    } catch (error) {
      setState(prev => ({
        ...prev,
        error: error instanceof Error ? error.message : 'Complete module failed',
      }));
    }
  };

  const completeLab = async (labId: string) => {
    try {
      if (!state.session) throw new Error('Not authenticated');

      const response = await fetch('/api/auth/complete-lab', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ labId }),
        credentials: 'include', // Include session cookies
      });

      if (!response.ok) {
        throw new Error('Failed to complete lab');
      }

      const result = await response.json();

      // Update local state
      const updatedUser = { completedLabs: result.completedLabs };
      setState(prev => ({
        ...prev,
        user: prev.user ? { ...prev.user, ...updatedUser } : null,
        session: prev.session ? { ...prev.session, user: { ...prev.session.user, ...updatedUser } } : null
      }));
    } catch (error) {
      setState(prev => ({
        ...prev,
        error: error instanceof Error ? error.message : 'Complete lab failed',
      }));
    }
  };

  return (
    <AuthContext.Provider value={{ 
      state, 
      signIn, 
      signUp, 
      signOut, 
      updateProfile,
      updateProgress,
      completeModule,
      completeLab
    }}>
      {children}
    </AuthContext.Provider>
  );
};

// Custom hook to use auth context
export const useAuth = () => {
  const context = useContext(AuthContext);
  if (!context) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
};