"""
User model for PostgreSQL database and Pydantic schemas
"""

from sqlalchemy import Column, String, DateTime, Enum
from sqlalchemy.dialects.postgresql import UUID
from datetime import datetime
import uuid
import enum
from app.database.connection import Base
from pydantic import BaseModel, EmailStr, Field
from typing import Optional


class SoftwareExperience(str, enum.Enum):
    """Software experience levels"""

    BEGINNER = "beginner"
    INTERMEDIATE = "intermediate"
    ADVANCED = "advanced"


class HardwareExperience(str, enum.Enum):
    """Hardware experience levels"""

    NONE = "none"
    HOBBYIST = "hobbyist"
    PROFESSIONAL = "professional"


class User(Base):
    """User model with authentication and background information"""

    __tablename__ = "users"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    email = Column(String(255), unique=True, nullable=False, index=True)
    hashed_password = Column(String(255), nullable=False)
    software_experience = Column(Enum(SoftwareExperience), nullable=True)
    hardware_experience = Column(Enum(HardwareExperience), nullable=True)
    created_at = Column(DateTime(timezone=True), default=datetime.utcnow, nullable=False)

    def __repr__(self):
        return f"<User(id={self.id}, email={self.email})>"


# --- Pydantic Schemas ---

class UserBase(BaseModel):
    email: EmailStr
    software_experience: Optional[SoftwareExperience] = None
    hardware_experience: Optional[HardwareExperience] = None


class UserCreate(UserBase):
    password: str = Field(..., min_length=8)


class UserUpdate(BaseModel):
    software_experience: Optional[SoftwareExperience] = None
    hardware_experience: Optional[HardwareExperience] = None


class UserResponse(UserBase):
    id: uuid.UUID
    created_at: datetime

    class Config:
        from_attributes = True


class UserLogin(BaseModel):
    email: EmailStr
    password: str


class Token(BaseModel):


    access_token: str


    token_type: str








class AuthResponse(BaseModel):


    access_token: str


    token_type: str


    user: UserResponse

