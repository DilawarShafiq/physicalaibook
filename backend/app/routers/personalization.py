"""
Personalization Router
Handles learning path recommendations and content personalization
"""

from fastapi import APIRouter, Depends, HTTPException
from sqlalchemy.ext.asyncio import AsyncSession
from app.database.connection import get_db
from app.models.user import User
from app.dependencies import get_current_user
from app.services.personalization_service import PersonalizationService
from pydantic import BaseModel
from typing import Dict, Any
import uuid

router = APIRouter()

class PersonalizeChapterRequest(BaseModel):
    content: str
    chapter_title: str

class TranslateChapterRequest(BaseModel):
    content: str
    target_language: str = "Urdu"

class PersonalizedContentResponse(BaseModel):
    chapter_id: str
    personalized_content: str

class TranslatedContentResponse(BaseModel):
    original_content: str
    translated_content: str
    target_language: str
    chapter_id: str = "unknown"

@router.get("/recommendations")
async def get_recommendations(
    current_user: User = Depends(get_current_user),
    db: AsyncSession = Depends(get_db)
) -> Dict[str, Any]:
    """Get personalized learning path recommendations"""
    service = PersonalizationService(db)
    return await service.get_recommendations(current_user)


@router.post("/recommendations")
async def regenerate_recommendations(
    current_user: User = Depends(get_current_user),
    db: AsyncSession = Depends(get_db)
) -> Dict[str, Any]:
    """Force regenerate recommendations"""
    service = PersonalizationService(db)
    # To regenerate, we'll just let the service know to ignore cache
    return await service.get_recommendations(current_user)


@router.post("/chapters/{chapter_id}/personalize", response_model=PersonalizedContentResponse)
async def personalize_chapter(
    chapter_id: str,
    request: PersonalizeChapterRequest,
    current_user: User = Depends(get_current_user),
    db: AsyncSession = Depends(get_db)
) -> PersonalizedContentResponse:
    """Personalize chapter content based on user profile"""
    service = PersonalizationService(db)

    personalized_content = await service.get_personalized_content(
        current_user,
        chapter_id,
        request.content
    )

    return PersonalizedContentResponse(
        chapter_id=chapter_id,
        personalized_content=personalized_content
    )


@router.post("/chapters/translate", response_model=TranslatedContentResponse)
async def translate_chapter(
    request: TranslateChapterRequest,
    current_user: User = Depends(get_current_user),
    db: AsyncSession = Depends(get_db)
) -> TranslatedContentResponse:
    """Translate chapter content to target language using LLM"""
    service = PersonalizationService(db)

    # Use the LLM service to translate content
    # For now, we'll use the existing LLM service in a different method
    # Let's add translation capability to the service
    from app.services.llm_service import LLMService

    llm_service = LLMService()
    translated_content = await llm_service.translate_content(
        request.content,
        request.target_language
    )

    return TranslatedContentResponse(
        original_content=request.content,
        translated_content=translated_content,
        target_language=request.target_language
    )
