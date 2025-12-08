"""
Custom exception classes for the application
"""

from typing import Optional


class AppException(Exception):
    """Base exception class for application errors"""

    def __init__(
        self,
        message: str,
        status_code: int = 500,
        error_code: Optional[str] = None,
    ):
        self.message = message
        self.status_code = status_code
        self.error_code = error_code or "INTERNAL_ERROR"
        super().__init__(self.message)


class AuthenticationError(AppException):
    """Authentication failed"""

    def __init__(self, message: str = "Authentication failed"):
        super().__init__(message, status_code=401, error_code="AUTHENTICATION_ERROR")


class AuthorizationError(AppException):
    """User not authorized for this action"""

    def __init__(self, message: str = "Not authorized"):
        super().__init__(message, status_code=403, error_code="AUTHORIZATION_ERROR")


class NotFoundError(AppException):
    """Resource not found"""

    def __init__(self, message: str = "Resource not found"):
        super().__init__(message, status_code=404, error_code="NOT_FOUND")


class ValidationError(AppException):
    """Input validation failed"""

    def __init__(self, message: str = "Validation failed"):
        super().__init__(message, status_code=422, error_code="VALIDATION_ERROR")


class RateLimitError(AppException):
    """Rate limit exceeded"""

    def __init__(self, message: str = "Rate limit exceeded"):
        super().__init__(message, status_code=429, error_code="RATE_LIMIT_EXCEEDED")


class ExternalServiceError(AppException):
    """External service (OpenAI, Qdrant, etc.) error"""

    def __init__(self, message: str = "External service error", service: Optional[str] = None):
        error_code = f"{service.upper()}_ERROR" if service else "EXTERNAL_SERVICE_ERROR"
        super().__init__(message, status_code=502, error_code=error_code)
