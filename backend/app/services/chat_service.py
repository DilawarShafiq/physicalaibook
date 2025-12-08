"""
Chat Service
Manages chat conversations and message persistence
"""

from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy import select
from fastapi import HTTPException, status
from app.models.chat import ChatConversation, ChatMessage, MessageSource
from app.models.user import User
from app.services.rag_service import RAGService
from datetime import datetime, timedelta
import uuid
from typing import List, Optional

class ChatService:
    def __init__(self, db: AsyncSession):
        self.db = db
        self.rag_service = RAGService()

    async def create_conversation(self, user: User) -> ChatConversation:
        """Create a new conversation"""
        conversation = ChatConversation(
            user_id=user.id,
            expires_at=datetime.utcnow() + timedelta(hours=24),
            message_count=0
        )
        self.db.add(conversation)
        await self.db.commit()
        await self.db.refresh(conversation)
        return conversation

    async def get_conversations(self, user: User, limit: int = 20) -> List[ChatConversation]:
        """List conversations for user"""
        query = select(ChatConversation).where(
            ChatConversation.user_id == user.id
        ).order_by(ChatConversation.created_at.desc()).limit(limit)
        
        result = await self.db.execute(query)
        return result.scalars().all()

    async def get_conversation(self, conversation_id: uuid.UUID, user: User) -> ChatConversation:
        """Get single conversation and verify ownership"""
        query = select(ChatConversation).where(
            ChatConversation.id == conversation_id,
            ChatConversation.user_id == user.id
        )
        result = await self.db.execute(query)
        conversation = result.scalar_one_or_none()
        
        if not conversation:
            raise HTTPException(status_code=404, detail="Conversation not found")
            
        return conversation

    async def get_messages(self, conversation_id: uuid.UUID, user: User) -> List[ChatMessage]:
        """Get messages for a conversation"""
        # First verify ownership
        await self.get_conversation(conversation_id, user)
        
        query = select(ChatMessage).where(
            ChatMessage.conversation_id == conversation_id
        ).order_by(ChatMessage.created_at.asc())
        
        result = await self.db.execute(query)
        return result.scalars().all()

    async def process_user_message(
        self, 
        conversation_id: uuid.UUID, 
        user: User, 
        content: str,
        selected_text: Optional[str] = None
    ) -> ChatMessage:
        """
        Process user message:
        1. Save user message
        2. Generate AI response
        3. Save AI message
        4. Return AI message
        """
        conversation = await self.get_conversation(conversation_id, user)
        
        # 1. Save user message
        user_msg = ChatMessage(
            conversation_id=conversation.id,
            role="user",
            content=content,
            selected_text=selected_text
        )
        self.db.add(user_msg)
        
        # Get history for context
        history_msgs = await self.get_messages(conversation_id, user)
        history = [{"role": m.role, "content": m.content} for m in history_msgs]

        # 2. Generate AI response
        response_text, sources = await self.rag_service.generate_response(
            query=content, 
            history=history,
            selected_text=selected_text
        )

        # 3. Save AI message
        ai_msg = ChatMessage(
            conversation_id=conversation.id,
            role="assistant",
            content=response_text,
            sources=sources
        )
        self.db.add(ai_msg)
        
        # Update conversation
        conversation.message_count += 2
        
        await self.db.commit()
        await self.db.refresh(ai_msg)
        
        return ai_msg
