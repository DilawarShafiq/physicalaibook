"""
Chat models for PostgreSQL and Pydantic schemas
"""

from sqlalchemy import Column, String, DateTime, ForeignKey, Integer, Text
from sqlalchemy.dialects.postgresql import UUID, JSONB
from sqlalchemy.orm import relationship
from datetime import datetime, timedelta
import uuid
from app.database.connection import Base
from pydantic import BaseModel, Field
from typing import Optional, List, Any


class ChatConversation(Base):
    """Chat conversation model"""

    __tablename__ = "chat_conversations"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    user_id = Column(UUID(as_uuid=True), ForeignKey("users.id"), nullable=False, index=True)
    title = Column(String(255), nullable=True)
    created_at = Column(DateTime(timezone=True), default=datetime.utcnow, nullable=False)
    expires_at = Column(DateTime(timezone=True), nullable=False)
    message_count = Column(Integer, default=0)

    # Relationship
    messages = relationship("ChatMessage", back_populates="conversation", cascade="all, delete-orphan")

    def __repr__(self):
        return f"<ChatConversation(id={self.id}, user_id={self.user_id})>"


class ChatMessage(Base):
    """Chat message model"""

    __tablename__ = "chat_messages"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    conversation_id = Column(UUID(as_uuid=True), ForeignKey("chat_conversations.id"), nullable=False, index=True)
    role = Column(String(50), nullable=False)  # user, assistant
    content = Column(Text, nullable=False)
    sources = Column(JSONB, nullable=True)
    selected_text = Column(Text, nullable=True)  # For US5
    created_at = Column(DateTime(timezone=True), default=datetime.utcnow, nullable=False)

    conversation = relationship("ChatConversation", back_populates="messages")

    def __repr__(self):
        return f"<ChatMessage(id={self.id}, role={self.role})>"


# --- Pydantic Schemas ---

class MessageSource(BaseModel):
    content: str
    metadata: Optional[dict] = None


class ChatMessageBase(BaseModel):
    role: str
    content: str


class ChatMessageCreate(ChatMessageBase):
    selected_text: Optional[str] = None


class ChatMessageResponse(ChatMessageBase):
    id: uuid.UUID
    conversation_id: uuid.UUID
    sources: Optional[List[MessageSource]] = None
    created_at: datetime
    selected_text: Optional[str] = None

    class Config:
        from_attributes = True


class SendMessageRequest(BaseModel):
    content: str
    selected_text: Optional[str] = None


class ChatConversationResponse(BaseModel):
    id: uuid.UUID
    title: Optional[str] = None
    created_at: datetime
    message_count: int

    class Config:
        from_attributes = True