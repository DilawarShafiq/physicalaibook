"""
Authentication Service
Handles user registration and authentication logic
"""

from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy import select
from fastapi import HTTPException, status
from app.models.user import User, UserCreate, UserLogin
from app.utils.password import hash_password, verify_password
from app.utils.jwt import create_access_token


class AuthService:
    def __init__(self, db: AsyncSession):
        self.db = db

    async def create_user(self, user_in: UserCreate) -> User:
        """
        Create a new user.
        Raises exception if email already exists.
        """
        # Check if user exists
        query = select(User).where(User.email == user_in.email)
        result = await self.db.execute(query)
        existing_user = result.scalar_one_or_none()

        if existing_user:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Email already registered"
            )

        # Create new user
        new_user = User(
            email=user_in.email,
            hashed_password=hash_password(user_in.password),
            software_experience=user_in.software_experience,
            hardware_experience=user_in.hardware_experience
        )

        self.db.add(new_user)
        await self.db.commit()
        await self.db.refresh(new_user)

        return new_user

    async def authenticate_user(self, login_data: UserLogin) -> User:
        """
        Authenticate a user by email and password.
        Returns the user if successful, raises exception otherwise.
        """
        query = select(User).where(User.email == login_data.email)
        result = await self.db.execute(query)
        user = result.scalar_one_or_none()

        if not user:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Incorrect email or password",
                headers={"WWW-Authenticate": "Bearer"},
            )

        if not verify_password(login_data.password, user.hashed_password):
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Incorrect email or password",
                headers={"WWW-Authenticate": "Bearer"},
            )

        return user
