import React, { useState, useEffect } from 'react';
import { useAuth } from '../../contexts/AuthContext';
import apiClient from '../../api/client';
import { SoftwareExperience, HardwareExperience } from '../../types';

export default function ProfileForm() {
  const { user, refreshUser } = useAuth();
  const [softwareExp, setSoftwareExp] = useState<SoftwareExperience>('beginner');
  const [hardwareExp, setHardwareExp] = useState<HardwareExperience>('none');
  const [message, setMessage] = useState('');
  const [error, setError] = useState('');
  const [loading, setLoading] = useState(false);

  useEffect(() => {
    if (user) {
      if (user.software_experience) setSoftwareExp(user.software_experience);
      if (user.hardware_experience) setHardwareExp(user.hardware_experience);
    }
  }, [user]);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setMessage('');
    setError('');
    setLoading(true);

    try {
      await apiClient.updateProfile({
        software_experience: softwareExp,
        hardware_experience: hardwareExp,
      });
      await refreshUser();
      setMessage('Profile updated successfully!');
    } catch (err: any) {
      setError(err.message || 'Failed to update profile');
    } finally {
      setLoading(false);
    }
  };

  if (!user) return null;

  return (
    <div className="profile-form-container" style={{
      background: 'var(--ifm-background-surface-color)',
      padding: '2rem',
      borderRadius: '16px',
      boxShadow: '0 4px 12px rgba(0,0,0,0.05)',
      border: '1px solid var(--ifm-toc-border-color)',
      maxWidth: '600px',
      margin: '1rem auto'
    }}>
      <div style={{ marginBottom: '1.5rem' }}>
        <h3 style={{
          margin: 0,
          fontSize: '1.4rem',
          fontWeight: '600',
          color: 'var(--ifm-heading-color)',
          display: 'flex',
          alignItems: 'center',
          gap: '0.5rem'
        }}>
          <span>👤</span>
          <span>Your Learning Profile</span>
        </h3>
        <p style={{
          margin: '0.5rem 0 0',
          color: 'var(--ifm-color-emphasis-700)',
          fontSize: '1rem'
        }}>
          We use this information to personalize your learning experience and recommend appropriate content.
        </p>
      </div>

      {message && (
        <div className="alert alert--success" style={{
          marginBottom: '1.5rem',
          padding: '0.75rem',
          borderRadius: '8px',
          background: '#efe',
          border: '1px solid #cfc',
          color: '#2e7d32'
        }}>
          {message}
        </div>
      )}

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
              transition: 'border-color 0.2s ease',
              maxWidth: '400px'
            }}
            onFocus={(e) => e.target.style.borderColor = 'var(--ifm-color-primary)'}
            onBlur={(e) => e.target.style.borderColor = 'var(--ifm-toc-border-color)'}
          >
            <option value="beginner">Beginner - New to programming or robotics</option>
            <option value="intermediate">Intermediate - Familiar with programming concepts</option>
            <option value="advanced">Advanced - Experienced with robotics frameworks</option>
          </select>
          <div style={{ fontSize: '0.85rem', color: 'var(--ifm-color-emphasis-600)', marginTop: '0.5rem' }}>
            We'll customize content complexity based on your experience level
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
              transition: 'border-color 0.2s ease',
              maxWidth: '400px'
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
          className="button button--primary"
          style={{
            padding: '0.8rem 1.5rem',
            borderRadius: '8px',
            fontSize: '1rem',
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
              <span>Saving...</span>
            </span>
          ) : (
            <span style={{ display: 'flex', alignItems: 'center', justifyContent: 'center', gap: '0.5rem' }}>
              <span>💾</span>
              <span>Update Profile</span>
            </span>
          )}
        </button>
      </form>
    </div>
  );
}
