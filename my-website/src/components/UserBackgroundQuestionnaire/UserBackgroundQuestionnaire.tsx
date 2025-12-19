import React, { useState, useEffect } from 'react';
import { useSession, useAuth } from '@site/src/auth/client';

interface UserBackgroundData {
  expertiseLevel: string;
  background: string;
  softwareExperience: string;
  hardwareExperience: string;
}

const UserBackgroundQuestionnaire: React.FC = () => {
  const { data: session } = useSession();
  const { update } = useAuth();
  const [showQuestionnaire, setShowQuestionnaire] = useState(false);
  const [formData, setFormData] = useState<UserBackgroundData>({
    expertiseLevel: '',
    background: '',
    softwareExperience: '',
    hardwareExperience: ''
  });
  const [loading, setLoading] = useState(false);

  // Check if user has already completed the questionnaire
  useEffect(() => {
    if (session?.user) {
      // Check if user has already filled out background info
      const hasBackgroundInfo = session.user.expertiseLevel ||
                               session.user.background ||
                               session.user.softwareExperience ||
                               session.user.hardwareExperience;

      if (!hasBackgroundInfo) {
        setShowQuestionnaire(true);
      }
    }
  }, [session]);

  const handleChange = (e: React.ChangeEvent<HTMLInputElement | HTMLSelectElement | HTMLTextAreaElement>) => {
    const { name, value } = e.target;
    setFormData(prev => ({
      ...prev,
      [name]: value
    }));
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setLoading(true);

    try {
      // Update user profile with background information
      await update({
        data: {
          expertiseLevel: formData.expertiseLevel,
          background: formData.background,
          softwareExperience: formData.softwareExperience,
          hardwareExperience: formData.hardwareExperience
        }
      });

      setShowQuestionnaire(false);
    } catch (error) {
      console.error('Error updating user background:', error);
    } finally {
      setLoading(false);
    }
  };

  if (!showQuestionnaire || !session) return null;

  return (
    <div className="user-background-questionnaire">
      <div className="questionnaire-overlay">
        <div className="questionnaire-modal">
          <h2>Tell us about your background</h2>
          <p>Help us personalize your experience by sharing your expertise</p>

          <form onSubmit={handleSubmit}>
            <div className="form-group">
              <label htmlFor="expertiseLevel">Expertise Level:</label>
              <select
                id="expertiseLevel"
                name="expertiseLevel"
                value={formData.expertiseLevel}
                onChange={handleChange}
                required
              >
                <option value="">Select your level</option>
                <option value="beginner">Beginner</option>
                <option value="intermediate">Intermediate</option>
                <option value="advanced">Advanced</option>
                <option value="expert">Expert</option>
              </select>
            </div>

            <div className="form-group">
              <label htmlFor="background">Background:</label>
              <select
                id="background"
                name="background"
                value={formData.background}
                onChange={handleChange}
                required
              >
                <option value="">Select your background</option>
                <option value="software">Software Engineering</option>
                <option value="hardware">Hardware Engineering</option>
                <option value="data-science">Data Science</option>
                <option value="product">Product Management</option>
                <option value="design">Design</option>
                <option value="business">Business/Management</option>
                <option value="student">Student</option>
                <option value="other">Other</option>
              </select>
            </div>

            <div className="form-group">
              <label htmlFor="softwareExperience">Software Experience:</label>
              <textarea
                id="softwareExperience"
                name="softwareExperience"
                value={formData.softwareExperience}
                onChange={handleChange}
                placeholder="Describe your software development experience..."
                rows={3}
              />
            </div>

            <div className="form-group">
              <label htmlFor="hardwareExperience">Hardware Experience:</label>
              <textarea
                id="hardwareExperience"
                name="hardwareExperience"
                value={formData.hardwareExperience}
                onChange={handleChange}
                placeholder="Describe your hardware experience..."
                rows={3}
              />
            </div>

            <div className="form-actions">
              <button
                type="submit"
                disabled={loading}
                className="submit-btn"
              >
                {loading ? 'Saving...' : 'Complete Profile'}
              </button>
            </div>
          </form>
        </div>
      </div>
    </div>
  );
};

export default UserBackgroundQuestionnaire;