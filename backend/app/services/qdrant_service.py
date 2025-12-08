"""
Qdrant Service
Handles vector similarity search and retrieval
"""

from qdrant_client.http import models
from app.database.qdrant import get_async_qdrant_client
from app.config import settings
from typing import List, Dict, Any, Optional


class QdrantService:
    def __init__(self):
        self.client = get_async_qdrant_client()
        self.collection_name = settings.QDRANT_COLLECTION_NAME

    async def search_similar(
        self, 
        vector: List[float], 
        limit: int = 5, 
        score_threshold: float = 0.7,
        filters: Optional[Dict[str, Any]] = None
    ) -> List[models.ScoredPoint]:
        """
        Search for similar vectors in Qdrant
        """
        query_filter = None
        if filters:
            # Build Qdrant filter from dictionary
            must_conditions = []
            for key, value in filters.items():
                must_conditions.append(
                    models.FieldCondition(
                        key=key,
                        match=models.MatchValue(value=value)
                    )
                )
            if must_conditions:
                query_filter = models.Filter(must=must_conditions)

        results = await self.client.search(
            collection_name=self.collection_name,
            query_vector=vector,
            limit=limit,
            score_threshold=score_threshold,
            query_filter=query_filter
        )
        return results

    async def create_collection_if_not_exists(self, vector_size: int = 1536):
        """
        Ensure collection exists
        """
        collections = await self.client.get_collections()
        exists = any(c.name == self.collection_name for c in collections.collections)
        
        if not exists:
            await self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(
                    size=vector_size,
                    distance=models.Distance.COSINE
                )
            )
