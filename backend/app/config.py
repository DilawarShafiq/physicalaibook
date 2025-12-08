"""
Configuration Management
Loads environment variables and provides application settings
"""

from pydantic_settings import BaseSettings, SettingsConfigDict
from typing import List


class Settings(BaseSettings):
    """Application settings loaded from environment variables"""

    model_config = SettingsConfigDict(env_file=".env", env_file_encoding="utf-8", extra="ignore")

    # Database
    DATABASE_URL: str
    DATABASE_ECHO: bool = False

    # Qdrant Vector Database
    QDRANT_URL: str = "http://localhost:6333"  # Default Qdrant URL
    QDRANT_API_KEY: str = ""  # Optional
    QDRANT_COLLECTION_NAME: str = "textbook_embeddings"

    # Google Gemini Settings (instead of OpenAI)
    GOOGLE_API_KEY: str
    GEMINI_MODEL: str = "gemini-2.5-flash"

    # JWT Authentication
    SECRET_KEY: str
    ALGORITHM: str = "HS256"
    ACCESS_TOKEN_EXPIRE_MINUTES: int = 10080  # 7 days

    # CORS
    CORS_ORIGINS: List[str] = ["http://localhost:3000", "http://127.0.0.1:3000", "http://localhost:3001"]

    # Application Settings
    ENVIRONMENT: str = "development"
    LOG_LEVEL: str = "INFO"
    API_V1_PREFIX: str = "/api/v1"

    # Rate Limiting
    MAX_PERSONALIZATIONS_PER_DAY: int = 5
    MAX_CHAT_MESSAGES_PER_MINUTE: int = 10

    # Content Settings
    CHUNK_SIZE: int = 1024
    CHUNK_OVERLAP_PERCENTAGE: int = 15

    @property
    def chunk_overlap(self) -> int:
        """Calculate chunk overlap in tokens"""
        return int(self.CHUNK_SIZE * self.CHUNK_OVERLAP_PERCENTAGE / 100)


# Global settings instance
settings = Settings()
