import React, { useState, useEffect } from 'react';
import { NavbarSecondaryMenuFiller } from '@docusaurus/theme-common';
import { useLocation } from '@docusaurus/router';
import { useAuth } from '../components/auth/client';
import LoginModal from '../components/auth/LoginModal';

const UserMenu = () => {
  const { state, signOut } = useAuth();
  const [isModalOpen, setIsModalOpen] = useState(false);
  const [isLogin, setIsLogin] = useState(true);
  const location = useLocation();

  // Close modal on route change
  useEffect(() => {
    setIsModalOpen(false);
  }, [location]);

  return (
    <>
      {state.user ? (
        <div className="navbar__item">
          <div className="dropdown dropdown--right dropdown--navbar">
            <button 
              className="navbar__link dropdown__trigger"
              aria-label="User menu"
            >
              {state.user.name || state.user.email}
            </button>
            <ul className="dropdown__menu">
              <li>
                <a className="dropdown__link" href="/profile">
                  Profile
                </a>
              </li>
              <li>
                <button 
                  className="dropdown__link dropdown__link--container"
                  onClick={signOut}
                >
                  Sign Out
                </button>
              </li>
            </ul>
          </div>
        </div>
      ) : (
        <div className="navbar__item">
          <button 
            className="button button--primary"
            onClick={() => setIsModalOpen(true)}
          >
            Sign In
          </button>
        </div>
      )}
      
      <LoginModal 
        isOpen={isModalOpen} 
        onClose={() => setIsModalOpen(false)} 
      />
    </>
  );
};

const UserMenuWrapper = (props) => {
  return <NavbarSecondaryMenuFiller component={UserMenu} props={props} />;
};

export default UserMenuWrapper;