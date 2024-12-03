"""Exceptions for stepper motor protocol."""

from logging import getLogger

logger = getLogger(__name__)


class MotorError(Exception):
    """Base exception for motor errors."""

    def __init__(self, message: str = "Unspecified"):
        """Initialize the exception."""
        logger.error(message)


class CommandError(MotorError):
    """Error executing a command."""


class ValidationError(MotorError):
    """Error validating command parameters."""


class CommunicationError(MotorError):
    """Error communicating with the motor."""


class StatusError(MotorError):
    """Error with motor status."""
