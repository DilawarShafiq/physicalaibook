import React from 'react';
import Layout from '@theme/Layout';
import SignIn from '../components/Auth/SignIn';

export default function SignInPage() {
  return (
    <Layout title="Sign In" description="Sign in to your account">
      <div className="container margin-vert--lg">
        <SignIn />
      </div>
    </Layout>
  );
}
