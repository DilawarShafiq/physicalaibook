"""
Personalization Service
Handles learning path recommendations and content personalization
"""

from sqlalchemy.ext.asyncio import AsyncSession
from app.models.user import User
from app.models.personalization import LearningPathRecommendation, PersonalizedContentCache
from sqlalchemy import select
from datetime import datetime, timedelta
from app.services.llm_service import LLMService
from typing import List, Optional
import uuid
import logging

logger = logging.getLogger(__name__)


class PersonalizationService:
    def __init__(self, db: AsyncSession):
        self.db = db
        self.llm_service = LLMService()

    async def get_recommendations(self, user: User) -> dict:
        """
        Generate personalized learning path recommendations based on user profile
        """
        # Check if we have cached recommendations
        query = select(LearningPathRecommendation).where(
            LearningPathRecommendation.user_id == user.id
        ).order_by(LearningPathRecommendation.created_at.desc()).limit(1)

        result = await self.db.execute(query)
        cached_rec = result.scalar_one_or_none()

        # If no cached recommendations or they're expired (24 hours), generate new ones
        if not cached_rec or cached_rec.expires_at < datetime.utcnow():
            recommendations = await self._generate_learning_path(user)
            
            # Create new recommendation entry
            new_rec = LearningPathRecommendation(
                user_id=user.id,
                recommended_chapters=recommendations,
                rationale=self._generate_rationale_for_user(user),
                experience_snapshot={
                    "software_experience": user.software_experience,
                    "hardware_experience": user.hardware_experience
                },
                expires_at=datetime.utcnow() + timedelta(hours=24)
            )
            self.db.add(new_rec)
            await self.db.commit()
            await self.db.refresh(new_rec)
            
            return {
                "recommendations": recommendations,
                "rationale": self._generate_rationale_for_user(user),
                "generated_at": new_rec.created_at.isoformat()
            }
        else:
            return {
                "recommendations": cached_rec.recommended_chapters,
                "rationale": cached_rec.rationale,
                "generated_at": cached_rec.created_at.isoformat()
            }

    async def _generate_learning_path(self, user: User) -> List[dict]:
        """
        Generate personalized learning path based on user profile
        """
        # This is a simplified example - in a real implementation, this would
        # analyze the textbook content and generate recommendations based on the user's background
        recommendations = []

        # Determine starting point based on software experience
        if user.software_experience == "beginner":
            recommendations.extend([
                {"chapter_id": "module-1/introduction", "title": "Introduction to ROS 2", "priority": 1},
                {"chapter_id": "module-1/basics", "title": "ROS 2 Basics", "priority": 2},
                {"chapter_id": "module-1/nodes-topics", "title": "Nodes and Topics", "priority": 3}
            ])
        elif user.software_experience == "intermediate":
            recommendations.extend([
                {"chapter_id": "module-1/services-actions", "title": "Services and Actions", "priority": 1},
                {"chapter_id": "module-2/simulation", "title": "Gazebo Simulation", "priority": 2},
                {"chapter_id": "module-3/perception", "title": "Perception Systems", "priority": 3}
            ])
        else:  # advanced
            recommendations.extend([
                {"chapter_id": "module-4/vla-integration", "title": "VLA Integration", "priority": 1},
                {"chapter_id": "module-3/ai-control", "title": "AI-Robot Control", "priority": 2},
                {"chapter_id": "module-2/advanced-simulation", "title": "Advanced Simulation", "priority": 3}
            ])

        # Adjust based on hardware experience
        if user.hardware_experience == "none":
            # Add more simulation-focused content
            recommendations.append({"chapter_id": "module-2/simulation", "title": "Simulation Fundamentals", "priority": 4})
        elif user.hardware_experience == "hobbyist":
            # Balance simulation and hardware
            recommendations.append({"chapter_id": "appendices/hardware-setup", "title": "Hardware Setup", "priority": 4})
        else:  # professional
            # Focus on advanced topics
            recommendations.append({"chapter_id": "module-4/hardware-integration", "title": "Real Hardware Integration", "priority": 4})

        return recommendations

    def _generate_rationale_for_user(self, user: User) -> str:
        """
        Generate rationale for why these recommendations were made
        """
        software_exp = user.software_experience or "unknown"
        hardware_exp = user.hardware_experience or "unknown"

        return f"""
        Based on your profile (Software: {software_exp}, Hardware: {hardware_exp}), 
        these recommendations are tailored to your experience level. 
        We suggest starting with fundamental concepts that match your background 
        and gradually building up to more advanced topics.
        """

    async def get_personalized_content(self, user: User, chapter_id: str, original_content: str) -> str:
        """
        Get personalized content for a chapter, either from cache or by generating it
        """
        # Check if we have cached personalized content
        query = select(PersonalizedContentCache).where(
            PersonalizedContentCache.user_id == user.id,
            PersonalizedContentCache.chapter_id == chapter_id,
            PersonalizedContentCache.software_experience == user.software_experience,
            PersonalizedContentCache.hardware_experience == user.hardware_experience
        )

        result = await self.db.execute(query)
        cached_content = result.scalar_one_or_none()

        if cached_content and cached_content.expires_at > datetime.utcnow():
            return cached_content.personalized_content

        # Generate personalized content
        personalized_content = await self.llm_service.personalize_content(
            original_content,
            user.software_experience,
            user.hardware_experience
        )

        # Cache the result
        if not cached_content:
            cached_content = PersonalizedContentCache(
                user_id=user.id,
                chapter_id=chapter_id,
                original_content=original_content,
                personalized_content=personalized_content,
                software_experience=user.software_experience,
                hardware_experience=user.hardware_experience,
                experience_snapshot={
                    "software_experience": user.software_experience,
                    "hardware_experience": user.hardware_experience
                },
                expires_at=datetime.utcnow() + timedelta(hours=24)
            )
            self.db.add(cached_content)
        else:
            cached_content.personalized_content = personalized_content
            cached_content.expires_at = datetime.utcnow() + timedelta(hours=24)

        await self.db.commit()
        return personalized_content