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
    <div className="profile-form-container">
      <h3>Your Background</h3>
      <p className="text--secondary">We use this to personalize your learning path.</p>
      
      {message && <div className="alert alert--success" style={{ marginBottom: '1rem' }}>{message}</div>}
      {error && <div className="alert alert--danger" style={{ marginBottom: '1rem' }}>{error}</div>}
      
      <form onSubmit={handleSubmit}>
        <div style={{ marginBottom: '1rem' }}>
          <label style={{ display: 'block', marginBottom: '0.5rem' }}>Software Experience</label>
          <select
            value={softwareExp}
            onChange={(e) => setSoftwareExp(e.target.value as SoftwareExperience)}
            style={{ width: '100%', padding: '0.5rem', maxWidth: '300px' }}
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
            style={{ width: '100%', padding: '0.5rem', maxWidth: '300px' }}
          >
            <option value="none">None</option>
            <option value="hobbyist">Hobbyist</option>
            <option value="professional">Professional</option>
          </select>
        </div>
        <button
          type="submit"
          disabled={loading}
          className="button button--secondary"
        >
          {loading ? 'Saving...' : 'Update Profile'}
        </button>
      </form>
    </div>
  );
}
