import React from 'react';
import { useAuth } from './client';

const UserMenu: React.FC = () => {
  const { state, signOut } = useAuth();

  if (state.loading) {
    return <div>Loading...</div>;
  }

  if (!state.user) {
    return null; // This component should only be used when user is authenticated
  }

  return (
    <div className="user-menu" style={{
      display: 'flex',
      alignItems: 'center',
      gap: '1rem'
    }}>
      <span>Hello, {state.user.name}!</span>
      <button 
        onClick={signOut}
        style={{
          padding: '0.5rem 1rem',
          backgroundColor: '#007cba',
          color: 'white',
          border: 'none',
          borderRadius: '4px',
          cursor: 'pointer'
        }}
      >
        Sign Out
      </button>
    </div>
  );
};

export default UserMenu;