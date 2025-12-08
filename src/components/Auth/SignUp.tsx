import React, { useState } from 'react';
import { useAuth } from '../../contexts/AuthContext';
import { useHistory } from '@docusaurus/router';
import { SoftwareExperience, HardwareExperience } from '../../types';

export default function SignUp() {
  const { signup } = useAuth();
  const history = useHistory();
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [softwareExp, setSoftwareExp] = useState<SoftwareExperience>('beginner');
  const [hardwareExp, setHardwareExp] = useState<HardwareExperience>('none');
  const [error, setError] = useState('');
  const [loading, setLoading] = useState(false);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError('');
    setLoading(true);

    try {
      await signup({
        email,
        password,
        software_experience: softwareExp,
        hardware_experience: hardwareExp,
      });
      history.push('/profile');
    } catch (err: any) {
      setError(err.message || 'Failed to sign up');
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className="auth-form-container" style={{ maxWidth: '400px', margin: '2rem auto', padding: '2rem', border: '1px solid #ddd', borderRadius: '8px' }}>
      <h2>Create Account</h2>
      {error && <div className="alert alert--danger" style={{ marginBottom: '1rem' }}>{error}</div>}
      <form onSubmit={handleSubmit}>
        <div style={{ marginBottom: '1rem' }}>
          <label style={{ display: 'block', marginBottom: '0.5rem' }}>Email</label>
          <input
            type="email"
            value={email}
            onChange={(e) => setEmail(e.target.value)}
            required
            style={{ width: '100%', padding: '0.5rem' }}
          />
        </div>
        <div style={{ marginBottom: '1rem' }}>
          <label style={{ display: 'block', marginBottom: '0.5rem' }}>Password (min 8 chars)</label>
          <input
            type="password"
            value={password}
            onChange={(e) => setPassword(e.target.value)}
            minLength={8}
            required
            style={{ width: '100%', padding: '0.5rem' }}
          />
        </div>
        <div style={{ marginBottom: '1rem' }}>
          <label style={{ display: 'block', marginBottom: '0.5rem' }}>Software Experience</label>
          <select
            value={softwareExp}
            onChange={(e) => setSoftwareExp(e.target.value as SoftwareExperience)}
            style={{ width: '100%', padding: '0.5rem' }}
          >
            <option value="beginner">Beginner</option>
            <option value="intermediate">Intermediate</option>
            <option value="advanced">Advanced</option>
          </select>
        </div>
        <div style={{ marginBottom: '1rem' }}>
          <label style={{ display: 'block', marginBottom: '0.5rem' }}>Hardware Experience</label>
          <select
            value={hardwareExp}
            onChange={(e) => setHardwareExp(e.target.value as HardwareExperience)}
            style={{ width: '100%', padding: '0.5rem' }}
          >
            <option value="none">None</option>
            <option value="hobbyist">Hobbyist</option>
            <option value="professional">Professional</option>
          </select>
        </div>
        <button
          type="submit"
          disabled={loading}
          className="button button--primary button--block"
        >
          {loading ? 'Creating Account...' : 'Sign Up'}
        </button>
      </form>
    </div>
  );
}
