"""
Qdrant Vector Database Connection
"""

from qdrant_client import QdrantClient, AsyncQdrantClient
from app.config import settings

def get_qdrant_client() -> QdrantClient:
    """
    Get synchronous Qdrant client
    Useful for scripts and blocking operations
    """
    return QdrantClient(
        url=settings.QDRANT_URL,
        api_key=settings.QDRANT_API_KEY,
    )

def get_async_qdrant_client() -> AsyncQdrantClient:
    """
    Get asynchronous Qdrant client
    Use this for API endpoints
    """
    return AsyncQdrantClient(
        url=settings.QDRANT_URL,
        api_key=settings.QDRANT_API_KEY,
    )
