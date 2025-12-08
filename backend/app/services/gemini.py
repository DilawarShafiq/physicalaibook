"""
Google Gemini Service
Handles AI operations using Google Gemini API
"""

import google.generativeai as genai
from app.config import settings
import logging

logger = logging.getLogger(__name__)

# Configure Gemini
genai.configure(api_key=settings.GOOGLE_API_KEY)


class GeminiService:
    """Service for interacting with Google Gemini API"""

    def __init__(self):
        self.model = genai.GenerativeModel(settings.GEMINI_MODEL)

    async def personalize_content(
        self,
        content: str,
        chapter_title: str,
        software_experience: str = "intermediate",
        hardware_experience: str = "hobbyist",
    ) -> str:
        """
        Personalize chapter content based on user background

        Args:
            content: Original chapter content
            chapter_title: Title of the chapter
            software_experience: User's software experience level
            hardware_experience: User's hardware experience level

        Returns:
            Personalized content
        """
        prompt = f"""You are an expert educator in Physical AI and Robotics. Personalize the following chapter content for a student with this background:

Software Experience: {software_experience}
Hardware Experience: {hardware_experience}

Chapter Title: {chapter_title}

Original Content:
{content}

Instructions:
1. Adjust the technical depth based on their software experience:
   - Beginner: Add more explanations, analogies, and step-by-step guidance
   - Intermediate: Balance theory and practice, focus on practical applications
   - Advanced: Include advanced topics, optimizations, and research insights

2. Tailor hardware examples based on their hardware experience:
   - None: Explain hardware concepts from scratch, use accessible analogies
   - Hobbyist: Reference common maker tools (Arduino, Raspberry Pi, 3D printing)
   - Professional: Include industrial standards, production considerations

3. Maintain the original structure and all code examples
4. Keep the same markdown formatting
5. Add personalized tips in callout boxes where relevant
6. Return ONLY the personalized content, no preamble

Personalized Content:
"""

        try:
            response = self.model.generate_content(prompt)
            return response.text
        except Exception as e:
            logger.error(f"Gemini personalization failed: {e}")
            raise

    async def translate_content(
        self, content: str, target_language: str = "Urdu", chapter_title: str = ""
    ) -> str:
        """
        Translate chapter content to target language

        Args:
            content: Original content in English
            target_language: Target language (default: Urdu)
            chapter_title: Title of the chapter

        Returns:
            Translated content
        """
        prompt = f"""Translate the following educational content about Physical AI and Robotics from English to {target_language}.

Chapter Title: {chapter_title}

Original Content:
{content}

Instructions:
1. Maintain all technical terms in English (e.g., ROS 2, Gazebo, NVIDIA Isaac)
2. Translate explanations and descriptions accurately
3. Preserve markdown formatting (headers, lists, code blocks, tables)
4. Keep code blocks unchanged
5. For Urdu: Use proper Urdu script and right-to-left formatting
6. Return ONLY the translated content, no preamble

Translated Content:
"""

        try:
            response = self.model.generate_content(prompt)
            return response.text
        except Exception as e:
            logger.error(f"Gemini translation failed: {e}")
            raise

    async def answer_question(
        self,
        question: str,
        context: str = "",
        conversation_history: list = None,
    ) -> dict:
        """
        Answer user question using RAG context

        Args:
            question: User's question
            context: Retrieved context from vector database
            conversation_history: Previous messages in conversation

        Returns:
            dict with answer and sources
        """
        history_text = ""
        if conversation_history:
            history_text = "\n".join(
                [
                    f"{'User' if msg['role'] == 'user' else 'Assistant'}: {msg['content']}"
                    for msg in conversation_history[-5:]  # Last 5 messages
                ]
            )

        prompt = f"""You are an AI teaching assistant for a Physical AI & Humanoid Robotics course. Answer the student's question using the provided context from the textbook.

{f"Conversation History:{history_text}" if history_text else ""}

Context from Textbook:
{context}

Student Question:
{question}

Instructions:
1. Answer based primarily on the provided context
2. If the context doesn't contain enough information, acknowledge this
3. Be clear, concise, and educational
4. Use examples where helpful
5. If relevant, suggest related topics to explore

Answer:
"""

        try:
            response = self.model.generate_content(prompt)
            return {"answer": response.text, "sources": []}
        except Exception as e:
            logger.error(f"Gemini question answering failed: {e}")
            raise


# Global service instance
gemini_service = GeminiService()
