"""
Embedding Service
Generates vector embeddings for text using OpenAI
"""

from openai import AsyncOpenAI
from app.config import settings
from typing import List

# Initialize OpenAI client
client = AsyncOpenAI(api_key=settings.OPENAI_API_KEY)


class EmbeddingService:
    async def get_embedding(self, text: str) -> List[float]:
        """
        Generate embedding for a single string of text
        """
        # Replace newlines to improve embedding performance
        text = text.replace("\n", " ")
        
        response = await client.embeddings.create(
            input=text,
            model=settings.OPENAI_EMBEDDING_MODEL
        )
        return response.data[0].embedding

    async def get_embeddings(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for a list of texts
        """
        # Replace newlines
        texts = [t.replace("\n", " ") for t in texts]
        
        response = await client.embeddings.create(
            input=texts,
            model=settings.OPENAI_EMBEDDING_MODEL
        )
        return [data.embedding for data in response.data]

