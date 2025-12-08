"""
Personalization models for PostgreSQL database
"""

from sqlalchemy import Column, String, DateTime, ForeignKey, Text, JSON
from sqlalchemy.dialects.postgresql import UUID
from datetime import datetime, timedelta
import uuid
from app.database.connection import Base


class LearningPathRecommendation(Base):
    """Learning path recommendations for users"""

    __tablename__ = "learning_path_recommendations"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    user_id = Column(UUID(as_uuid=True), ForeignKey("users.id", ondelete="CASCADE"), nullable=False, index=True)
    recommendations = Column(JSON, nullable=False)  # Array of recommended modules/topics
    created_at = Column(DateTime(timezone=True), default=datetime.utcnow, nullable=False)
    expires_at = Column(
        DateTime(timezone=True),
        default=lambda: datetime.utcnow() + timedelta(days=7),
        nullable=False,
    )

    def __repr__(self):
        return f"<LearningPathRecommendation(id={self.id}, user_id={self.user_id})>"


class PersonalizedContentCache(Base):
    """Cache for personalized chapter content"""

    __tablename__ = "personalized_content_cache"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    user_id = Column(UUID(as_uuid=True), ForeignKey("users.id", ondelete="CASCADE"), nullable=False)
    chapter_id = Column(String(255), nullable=False)
    personalized_content = Column(Text, nullable=False)
    created_at = Column(DateTime(timezone=True), default=datetime.utcnow, nullable=False)
    expires_at = Column(
        DateTime(timezone=True),
        default=lambda: datetime.utcnow() + timedelta(days=30),
        nullable=False,
    )

    def __repr__(self):
        return f"<PersonalizedContentCache(id={self.id}, user_id={self.user_id}, chapter_id={self.chapter_id})>"
