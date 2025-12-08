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
  const [showPassword, setShowPassword] = useState(false);

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
        <div style={{ fontSize: '2.5rem', marginBottom: '0.5rem' }}>🎓</div>
        <h2 style={{ margin: 0, fontSize: '1.5rem', fontWeight: '600', color: 'var(--ifm-heading-color)' }}>Create Your Account</h2>
        <p style={{ margin: '0.5rem 0 0', color: 'var(--ifm-color-emphasis-600)' }}>Join the Physical AI & Robotics community</p>
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
            Password <span style={{ fontSize: '0.8rem', color: 'var(--ifm-color-emphasis-600)' }}>(min 8 characters)</span>
          </label>
          <div style={{ position: 'relative' }}>
            <input
              type={showPassword ? "text" : "password"}
              value={password}
              onChange={(e) => setPassword(e.target.value)}
              minLength={8}
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

        <div style={{ marginBottom: '1.5rem' }}>
          <label style={{
            display: 'block',
            marginBottom: '0.5rem',
            fontWeight: '500',
            color: 'var(--ifm-color-emphasis-800)'
          }}>
            Software Experience Level
          </label>
          <select
            value={softwareExp}
            onChange={(e) => setSoftwareExp(e.target.value as SoftwareExperience)}
            style={{
              width: '100%',
              padding: '0.75rem 1rem',
              borderRadius: '8px',
              border: '2px solid var(--ifm-toc-border-color)',
              fontSize: '1rem',
              fontFamily: 'inherit',
              outline: 'none',
              background: 'var(--ifm-background-color)',
              cursor: 'pointer',
              transition: 'border-color 0.2s ease'
            }}
            onFocus={(e) => e.target.style.borderColor = 'var(--ifm-color-primary)'}
            onBlur={(e) => e.target.style.borderColor = 'var(--ifm-toc-border-color)'}
          >
            <option value="beginner">Beginner - New to programming or robotics</option>
            <option value="intermediate">Intermediate - Familiar with programming concepts</option>
            <option value="advanced">Advanced - Experienced with robotics frameworks</option>
          </select>
          <div style={{ fontSize: '0.85rem', color: 'var(--ifm-color-emphasis-600)', marginTop: '0.5rem' }}>
            We'll customize your learning experience based on your experience level
          </div>
        </div>

        <div style={{ marginBottom: '1.5rem' }}>
          <label style={{
            display: 'block',
            marginBottom: '0.5rem',
            fontWeight: '500',
            color: 'var(--ifm-color-emphasis-800)'
          }}>
            Hardware Experience Level
          </label>
          <select
            value={hardwareExp}
            onChange={(e) => setHardwareExp(e.target.value as HardwareExperience)}
            style={{
              width: '100%',
              padding: '0.75rem 1rem',
              borderRadius: '8px',
              border: '2px solid var(--ifm-toc-border-color)',
              fontSize: '1rem',
              fontFamily: 'inherit',
              outline: 'none',
              background: 'var(--ifm-background-color)',
              cursor: 'pointer',
              transition: 'border-color 0.2s ease'
            }}
            onFocus={(e) => e.target.style.borderColor = 'var(--ifm-color-primary)'}
            onBlur={(e) => e.target.style.borderColor = 'var(--ifm-toc-border-color)'}
          >
            <option value="none">None - First time with physical hardware</option>
            <option value="hobbyist">Hobbyist - Have worked on DIY electronics projects</option>
            <option value="professional">Professional - Have industry experience with hardware</option>
          </select>
          <div style={{ fontSize: '0.85rem', color: 'var(--ifm-color-emphasis-600)', marginTop: '0.5rem' }}>
            This helps us recommend appropriate projects and hardware requirements
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
              <span>Creating Account</span>
              <span>...</span>
            </span>
          ) : (
            <span style={{ display: 'flex', alignItems: 'center', justifyContent: 'center', gap: '0.5rem' }}>
              <span>🚀</span>
              <span>Create Account</span>
            </span>
          )}
        </button>

        <div style={{
          textAlign: 'center',
          marginTop: '1rem',
          fontSize: '0.9rem',
          color: 'var(--ifm-color-emphasis-700)'
        }}>
          Already have an account?{' '}
          <a
            href="/signin"
            style={{
              color: 'var(--ifm-color-primary)',
              textDecoration: 'none',
              fontWeight: '500'
            }}
          >
            Sign in here
          </a>
        </div>
      </form>
    </div>
  );
}
