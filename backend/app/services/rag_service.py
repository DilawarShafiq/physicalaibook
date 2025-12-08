"""
RAG Service
Orchestrates Retrieval Augmented Generation using Google Gemini
"""

from app.services.qdrant_service import QdrantService
import google.generativeai as genai
from app.config import settings
from typing import List, Dict, Any, Tuple
from google.generativeai.types import GenerationConfig

# Configure Google Generative AI
genai.configure(api_key=settings.GOOGLE_API_KEY)
model = genai.GenerativeModel(settings.GEMINI_MODEL)


class RAGService:
    def __init__(self):
        self.qdrant_service = QdrantService()

    async def generate_response(
        self,
        query: str,
        history: List[Dict[str, str]] = [],
        selected_text: str = None
    ) -> Tuple[str, List[Dict[str, Any]]]:
        """
        Generate AI response based on query and textbook context using Google Gemini.
        Returns (response_text, sources_list)
        """
        # 1. Retrieve relevant context from Qdrant
        # For now, use a simple keyword-based search (as embedding logic will need to be adapted)
        search_text = f"{selected_text}\n\nQuestion: {query}" if selected_text else query

        # This assumes QdrantService can perform a text-based search
        # In a real implementation, you'd need to handle embedding differently for Google models
        # We'll retrieve contexts related to the query
        search_results = await self.qdrant_service.search_similar(search_text, limit=3)

        # 2. Format context
        context_text = ""
        sources = []

        for i, result in enumerate(search_results):
            payload = result.payload
            content = payload.get("content", "")
            metadata = payload.get("metadata", {})

            context_text += f"Source {i+1}:\n{content}\n\n"
            sources.append({
                "content": content[:200] + "...", # Snippet
                "metadata": metadata
            })

        # 3. Construct System Prompt
        system_prompt = (
            "You are an expert AI teaching assistant for a Physical AI & Humanoid Robotics textbook. "
            "Answer the user's question using ONLY the provided context sources. "
            "If the answer is not in the context, say 'I cannot find the answer in the textbook.' "
            "Cite your sources by referring to 'Source 1', 'Source 2', etc. if applicable. "
            "Keep answers concise, educational, and encouraging."
        )

        if selected_text:
            system_prompt += f"\n\nThe user has selected the following text to ask about:\n'{selected_text}'\nFocus your answer on this text and the retrieved context."

        user_message = f"Context:\n{context_text}\n\nQuestion: {query}"

        # 4. Prepare messages with history
        full_prompt = f"{system_prompt}\n\n"

        # Add history if available
        for msg in history:  # Add all history messages
            full_prompt += f"{msg['role'].upper()}: {msg['content']}\n\n"

        full_prompt += f"USER: {user_message}\nASSISTANT:"

        try:
            # 5. Call Google Gemini
            response = await model.generate_content_async(
                full_prompt,
                generation_config=GenerationConfig(
                    temperature=0.3,  # Low temperature for factual grounding
                    max_output_tokens=2000
                )
            )

            response_text = response.text if response and response.text else "I cannot find the answer in the textbook."

            return response_text, sources

        except Exception as e:
            # If anything goes wrong, return a default response
            error_msg = "I cannot find the answer in the textbook."
            return error_msg, []
