"""
RAG Service
Orchestrates Retrieval Augmented Generation
"""

from app.services.embedding_service import EmbeddingService
from app.services.qdrant_service import QdrantService
from openai import AsyncOpenAI
from app.config import settings
from typing import List, Dict, Any, Tuple

client = AsyncOpenAI(api_key=settings.OPENAI_API_KEY)


class RAGService:
    def __init__(self):
        self.embedding_service = EmbeddingService()
        self.qdrant_service = QdrantService()

    async def generate_response(
        self, 
        query: str, 
        history: List[Dict[str, str]] = [],
        selected_text: str = None
    ) -> Tuple[str, List[Dict[str, Any]]]:
        """
        Generate AI response based on query and textbook context.
        Returns (response_text, sources_list)
        """
        # 1. Generate query embedding
        # If selected_text is provided, use it to bias the embedding or search
        search_text = f"{selected_text}\n\nQuestion: {query}" if selected_text else query
        query_vector = await self.embedding_service.get_embedding(search_text)

        # 2. Retrieve relevant context
        search_results = await self.qdrant_service.search_similar(query_vector, limit=3)
        
        # 3. Format context
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

        # 4. Construct System Prompt
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

        # 5. Call OpenAI
        messages = [{"role": "system", "content": system_prompt}]
        
        # Add limited history (last 4 messages)
        for msg in history[-4:]:
            messages.append({"role": msg["role"], "content": msg["content"]})
            
        messages.append({"role": "user", "content": user_message})

        completion = await client.chat.completions.create(
            model=settings.OPENAI_CHAT_MODEL,
            messages=messages,
            temperature=0.3, # Low temperature for factual grounding
        )

        response_text = completion.choices[0].message.content
        
        return response_text, sources
