"""
Authentication Router
Handles user signup, signin, profile management
"""

from fastapi import APIRouter, Depends, status, Body
from sqlalchemy.ext.asyncio import AsyncSession
from app.database.connection import get_db
from app.services.auth_service import AuthService
from app.models.user import UserCreate, UserLogin, UserResponse, AuthResponse, UserUpdate, User
from app.utils.jwt import create_access_token
from app.dependencies import get_current_user
from typing import Any

router = APIRouter()


@router.post("/signup", response_model=AuthResponse, status_code=status.HTTP_201_CREATED)
async def signup(
    user_in: UserCreate,
    db: AsyncSession = Depends(get_db)
) -> Any:
    """
    Create new user and return access token with user info
    """
    auth_service = AuthService(db)
    user = await auth_service.create_user(user_in)
    
    # Auto-login after signup
    access_token = create_access_token(data={"sub": str(user.id)})
    return {
        "access_token": access_token,
        "token_type": "bearer",
        "user": user
    }


@router.post("/signin", response_model=AuthResponse)
async def signin(
    login_data: UserLogin,
    db: AsyncSession = Depends(get_db)
) -> Any:
    """
    Authenticate user and return access token with user info
    """
    auth_service = AuthService(db)
    user = await auth_service.authenticate_user(login_data)
    
    access_token = create_access_token(data={"sub": str(user.id)})
    return {
        "access_token": access_token,
        "token_type": "bearer",
        "user": user
    }


@router.get("/me", response_model=UserResponse)
async def get_current_user_profile(
    current_user: User = Depends(get_current_user)
) -> Any:
    """
    Get current authenticated user profile
    """
    return current_user


@router.patch("/profile", response_model=UserResponse)
async def update_profile(
    user_update: UserUpdate,
    current_user: User = Depends(get_current_user),
    db: AsyncSession = Depends(get_db)
) -> Any:
    """
    Update user profile
    """
    if user_update.software_experience is not None:
        current_user.software_experience = user_update.software_experience
    if user_update.hardware_experience is not None:
        current_user.hardware_experience = user_update.hardware_experience
        
    db.add(current_user)
    await db.commit()
    await db.refresh(current_user)
    
    return current_user
