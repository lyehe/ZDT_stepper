"""Core module for stepper motor control.

This module provides the core functionality for stepper motors serial communication.

It includes constants, parameter classes, and exception handling for motor control operations.

Example:
    from stepper.core import DeviceParams, Speed, Direction

    # Create device parameters
    device = DeviceParams(port="COM4", baudrate=115200)

    # Configure movement parameters
    speed = Speed(1500)  # 1500 RPM
    direction = Direction.CW  # Clockwise rotation
"""

from . import configs, exceptions, parameters

__all__ = (
    *configs.__all__,
    *exceptions.__all__,
    *parameters.__all__,
)
