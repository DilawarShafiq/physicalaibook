import React from 'react';
import Layout from '@theme/Layout';
import ProfileForm from '../components/Auth/ProfileForm';
import ProtectedRoute from '../components/Auth/ProtectedRoute';
import { useAuth } from '../contexts/AuthContext';
import BrowserOnly from '@docusaurus/BrowserOnly';

function ProfileContent() {
  return (
    <BrowserOnly>
      {() => {
        const { user } = useAuth();
        return (
          <div className="container margin-vert--lg">
            <h1>My Profile</h1>
            <p>Manage settings for {user?.email}</p>
            <div style={{ marginTop: '2rem' }}>
              <ProfileForm />
            </div>
          </div>
        );
      }}
    </BrowserOnly>
  );
}

export default function ProfilePage() {
  return (
    <Layout title="Profile" description="Manage your profile">
      <ProtectedRoute>
        <ProfileContent />
      </ProtectedRoute>
    </Layout>
  );
}
