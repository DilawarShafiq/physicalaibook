import React from 'react';
import { useAuth } from '../../contexts/AuthContext';
import Link from '@docusaurus/Link';

export default function UserMenu() {
  const { user, signout } = useAuth();

  if (!user) {
    return (
      <div className="navbar__item">
        <Link to="/signin" className="button button--secondary button--sm margin-right--sm">Sign In</Link>
        <Link to="/signup" className="button button--primary button--sm">Sign Up</Link>
      </div>
    );
  }

  return (
    <div className="navbar__item dropdown dropdown--hoverable">
      <a className="navbar__link" href="#">{user.email}</a>
      <ul className="dropdown__menu">
        <li>
          <Link className="dropdown__link" to="/profile">My Profile</Link>
        </li>
        <li>
          <button 
            className="dropdown__link" 
            onClick={signout} 
            style={{ 
              background: 'none', 
              border: 'none', 
              cursor: 'pointer', 
              width: '100%', 
              textAlign: 'left',
              padding: '0.25rem 0.5rem'
            }}
          >
            Sign Out
          </button>
        </li>
      </ul>
    </div>
  );
}
