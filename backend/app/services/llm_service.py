"""
LLM Service
Handles interactions with Gemini via OpenAI Agents SDK
"""

from openai import OpenAI
from app.config import settings
from typing import Optional
import logging

logger = logging.getLogger(__name__)


class LLMService:
    def __init__(self):
        # Configure OpenAI client to use Gemini API
        self.client = OpenAI(
            api_key=settings.GOOGLE_API_KEY,
            base_url="https://generativelanguage.googleapis.com/v1beta/openai/"
        )
        self.model = settings.GEMINI_MODEL

    async def personalize_content(
        self,
        content: str,
        software_experience: str = "beginner",
        hardware_experience: str = "none"
    ) -> str:
        """
        Personalize content based on user experience levels using Google Gemini
        """
        system_prompt = f"""
        You are an expert educator for the Physical AI & Humanoid Robotics textbook.
        Your task is to adapt content based on the learner's experience level.

        The user has:
        - Software experience: {software_experience}
        - Hardware experience: {hardware_experience}

        Adapt the content accordingly:
        - For beginners: Include more explanations, analogies, and step-by-step instructions
        - For intermediate: Moderate level of detail, assume some background knowledge
        - For advanced: Concise explanations, focus on advanced concepts and implementation details

        For hardware experience:
        - None: Focus more on simulation and theory
        - Hobbyist: Include basic hardware integration examples
        - Professional: Include advanced hardware integration and optimization
        """

        user_message = f"""
        Please adapt the following content for the user's experience level:

        {content}
        """

        try:
            # Use OpenAI chat completions with Gemini
            response = self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_message}
                ],
                temperature=0.5,
                max_tokens=2000
            )

            if response and response.choices:
                return response.choices[0].message.content
            else:
                logger.warning("Gemini returned empty response, returning original content")
                return content

        except Exception as e:
            logger.error(f"Error personalizing content: {e}")
            # Return original content if personalization fails
            return content

    async def translate_content(
        self,
        content: str,
        target_language: str = "Urdu"
    ) -> str:
        """
        Translate content to target language using Google Gemini
        """
        system_prompt = f"""
        You are a professional translator specializing in technical content.
        Translate the following text to {target_language}.

        Guidelines:
        - Maintain technical accuracy for robotics and AI terminology
        - Preserve the educational tone and meaning
        - Keep code snippets and technical terms in English where appropriate
        - Format should be preserved as much as possible
        """

        user_message = f"""
        Please translate the following content to {target_language}:

        {content}
        """

        try:
            # Use OpenAI chat completions with Gemini
            response = self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_message}
                ],
                temperature=0.3,  # Lower temperature for more consistent translation
                max_tokens=3000
            )

            if response and response.choices:
                return response.choices[0].message.content
            else:
                logger.warning("Gemini returned empty response, returning original content")
                return content

        except Exception as e:
            logger.error(f"Error translating content: {e}")
            # Return original content if translation fails
            return content