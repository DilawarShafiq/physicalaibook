import React from 'react';
import Layout from '@theme/Layout';
import SignUp from '../components/Auth/SignUp';

export default function SignUpPage() {
  return (
    <Layout title="Sign Up" description="Create a new account">
      <div className="container margin-vert--lg">
        <SignUp />
      </div>
    </Layout>
  );
}
