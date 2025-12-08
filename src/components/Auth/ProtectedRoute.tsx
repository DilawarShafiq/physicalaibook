import React, { ReactNode } from 'react';
import { Redirect } from '@docusaurus/router';
import { useAuth } from '../../contexts/AuthContext';
import BrowserOnly from '@docusaurus/BrowserOnly';

interface ProtectedRouteProps {
  children: ReactNode;
}

export default function ProtectedRoute({ children }: ProtectedRouteProps) {
  return (
    <BrowserOnly>
      {() => {
        const { user, isLoading } = useAuth();
        
        if (isLoading) {
          return (
            <div style={{ display: 'flex', justifyContent: 'center', padding: '2rem' }}>
              <div className="loader">Loading...</div>
            </div>
          );
        }

        if (!user) {
          return <Redirect to="/signin" />;
        }

        return <>{children}</>;
      }}
    </BrowserOnly>
  );
}
