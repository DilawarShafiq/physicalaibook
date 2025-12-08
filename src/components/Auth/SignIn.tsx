import React, { useState } from 'react';
import { useAuth } from '../../contexts/AuthContext';
import { useHistory } from '@docusaurus/router';

export default function SignIn() {
  const { signin } = useAuth();
  const history = useHistory();
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [error, setError] = useState('');
  const [loading, setLoading] = useState(false);
  const [showPassword, setShowPassword] = useState(false);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError('');
    setLoading(true);

    try {
      await signin({ email, password });
      history.push('/profile');
    } catch (err: any) {
      setError(err.message || 'Failed to sign in');
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className="auth-form-container" style={{
      maxWidth: '450px',
      margin: '2rem auto',
      padding: '2rem',
      borderRadius: '16px',
      background: 'var(--ifm-background-surface-color)',
      boxShadow: '0 8px 30px rgba(0,0,0,0.1)',
      border: '1px solid var(--ifm-toc-border-color)',
      fontFamily: 'var(--ifm-font-family-base)'
    }}>
      <div style={{ textAlign: 'center', marginBottom: '1.5rem' }}>
        <div style={{ fontSize: '2.5rem', marginBottom: '0.5rem' }}>🔐</div>
        <h2 style={{ margin: 0, fontSize: '1.5rem', fontWeight: '600', color: 'var(--ifm-heading-color)' }}>Welcome Back</h2>
        <p style={{ margin: '0.5rem 0 0', color: 'var(--ifm-color-emphasis-600)' }}>Sign in to continue your robotics journey</p>
      </div>

      {error && (
        <div className="alert alert--danger" style={{
          marginBottom: '1.5rem',
          padding: '0.75rem',
          borderRadius: '8px',
          background: '#fee',
          border: '1px solid #fcc',
          color: '#d32f2f'
        }}>
          {error}
        </div>
      )}

      <form onSubmit={handleSubmit}>
        <div style={{ marginBottom: '1.5rem' }}>
          <label style={{
            display: 'block',
            marginBottom: '0.5rem',
            fontWeight: '500',
            color: 'var(--ifm-color-emphasis-800)'
          }}>
            Email Address
          </label>
          <input
            type="email"
            value={email}
            onChange={(e) => setEmail(e.target.value)}
            required
            style={{
              width: '100%',
              padding: '0.75rem 1rem',
              borderRadius: '8px',
              border: '2px solid var(--ifm-toc-border-color)',
              fontSize: '1rem',
              fontFamily: 'inherit',
              outline: 'none',
              transition: 'border-color 0.2s ease',
              background: 'var(--ifm-background-color)'
            }}
            onFocus={(e) => e.target.style.borderColor = 'var(--ifm-color-primary)'}
            onBlur={(e) => e.target.style.borderColor = 'var(--ifm-toc-border-color)'}
          />
        </div>

        <div style={{ marginBottom: '1.5rem' }}>
          <label style={{
            display: 'block',
            marginBottom: '0.5rem',
            fontWeight: '500',
            color: 'var(--ifm-color-emphasis-800)'
          }}>
            Password
          </label>
          <div style={{ position: 'relative' }}>
            <input
              type={showPassword ? "text" : "password"}
              value={password}
              onChange={(e) => setPassword(e.target.value)}
              required
              style={{
                width: '100%',
                padding: '0.75rem 3rem 0.75rem 1rem',
                borderRadius: '8px',
                border: '2px solid var(--ifm-toc-border-color)',
                fontSize: '1rem',
                fontFamily: 'inherit',
                outline: 'none',
                transition: 'border-color 0.2s ease',
                background: 'var(--ifm-background-color)'
              }}
              onFocus={(e) => e.target.style.borderColor = 'var(--ifm-color-primary)'}
              onBlur={(e) => e.target.style.borderColor = 'var(--ifm-toc-border-color)'}
            />
            <button
              type="button"
              onClick={() => setShowPassword(!showPassword)}
              style={{
                position: 'absolute',
                right: '0.75rem',
                top: '50%',
                transform: 'translateY(-50%)',
                background: 'none',
                border: 'none',
                cursor: 'pointer',
                color: 'var(--ifm-color-emphasis-600)',
                fontSize: '1rem'
              }}
            >
              {showPassword ? '👁️' : '👁️‍🗨️'}
            </button>
          </div>
        </div>

        <button
          type="submit"
          disabled={loading}
          className="button button--primary button--block"
          style={{
            width: '100%',
            padding: '0.9rem',
            borderRadius: '8px',
            fontSize: '1.1rem',
            fontWeight: '600',
            background: 'linear-gradient(135deg, var(--ifm-color-primary), #34a853)',
            border: 'none',
            cursor: loading ? 'not-allowed' : 'pointer',
            boxShadow: '0 4px 12px rgba(26, 115, 232, 0.3)',
            transition: 'all 0.2s ease',
          }}
          onMouseEnter={(e) => {
            if (!loading) {
              e.currentTarget.style.transform = 'translateY(-2px)';
              e.currentTarget.style.boxShadow = '0 6px 16px rgba(26, 115, 232, 0.4)';
            }
          }}
          onMouseLeave={(e) => {
            e.currentTarget.style.transform = 'translateY(0)';
            e.currentTarget.style.boxShadow = '0 4px 12px rgba(26, 115, 232, 0.3)';
          }}
        >
          {loading ? (
            <span style={{ display: 'flex', alignItems: 'center', justifyContent: 'center', gap: '0.5rem' }}>
              <span>Signing In</span>
              <span>...</span>
            </span>
          ) : (
            <span style={{ display: 'flex', alignItems: 'center', justifyContent: 'center', gap: '0.5rem' }}>
              <span>🔐</span>
              <span>Sign In</span>
            </span>
          )}
        </button>

        <div style={{
          textAlign: 'center',
          marginTop: '1rem',
          fontSize: '0.9rem',
          color: 'var(--ifm-color-emphasis-700)'
        }}>
          Don't have an account?{' '}
          <a
            href="/signup"
            style={{
              color: 'var(--ifm-color-primary)',
              textDecoration: 'none',
              fontWeight: '500'
            }}
          >
            Create one here
          </a>
        </div>
      </form>
    </div>
  );
}
