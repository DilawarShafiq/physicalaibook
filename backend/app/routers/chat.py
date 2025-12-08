"""
Chat Router
Handles RAG chatbot conversations and messages
"""

from fastapi import APIRouter, Depends, status, HTTPException
from sqlalchemy.ext.asyncio import AsyncSession
from app.database.connection import get_db
from app.services.chat_service import ChatService
from app.models.user import User
from app.models.chat import (
    ChatConversationResponse, 
    ChatMessageResponse, 
    SendMessageRequest
)
from app.dependencies import get_current_user
from typing import List, Any
import uuid

router = APIRouter()


@router.get("/conversations", response_model=List[ChatConversationResponse])
async def list_conversations(
    current_user: User = Depends(get_current_user),
    db: AsyncSession = Depends(get_db)
) -> Any:
    """List user conversations"""
    service = ChatService(db)
    return await service.get_conversations(current_user)


@router.post("/conversations", response_model=ChatConversationResponse, status_code=status.HTTP_201_CREATED)
async def create_conversation(
    current_user: User = Depends(get_current_user),
    db: AsyncSession = Depends(get_db)
) -> Any:
    """Create new conversation"""
    service = ChatService(db)
    return await service.create_conversation(current_user)


@router.get("/conversations/{conversation_id}/messages", response_model=List[ChatMessageResponse])
async def get_messages(
    conversation_id: uuid.UUID,
    current_user: User = Depends(get_current_user),
    db: AsyncSession = Depends(get_db)
) -> Any:
    """Get conversation messages"""
    service = ChatService(db)
    return await service.get_messages(conversation_id, current_user)


@router.post("/conversations/{conversation_id}/messages", response_model=ChatMessageResponse)
async def send_message(
    conversation_id: uuid.UUID,
    message_in: SendMessageRequest,
    current_user: User = Depends(get_current_user),
    db: AsyncSession = Depends(get_db)
) -> Any:
    """Send message to chatbot"""
    service = ChatService(db)
    return await service.process_user_message(
        conversation_id, 
        current_user, 
        message_in.content,
        message_in.selected_text
    )