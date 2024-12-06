"""Exceptions for stepper Stepper protocol."""

from logging import getLogger

logger = getLogger(__name__)

__all__ = [
    "StepperError",
    "CommandError",
    "ValidationError",
    "CommunicationError",
    "StatusError",
]


class StepperError(Exception):
    """Base exception for Stepper errors."""

    def __init__(self, message: str = "Unspecified"):
        """Initialize the exception."""
        logger.error(message)


class CommandError(StepperError):
    """Error executing a command."""


class ValidationError(StepperError):
    """Error validating command parameters."""


class CommunicationError(StepperError):
    """Error communicating with the Stepper."""


class StatusError(StepperError):
    """Error with Stepper status."""
