"""System commands for stepper motor."""

from stepper.commands.commands import (
    ReturnSuccess,
    TakeNoSetting,
    WithNoParams,
)
from stepper.stepper_core.configs import Code, Protocol

__all__ = [
    "CalibrateEncoder",
    "ZeroAllPositions",
    "ClearStall",
    "FactoryReset",
]


class SystemCommand(WithNoParams, TakeNoSetting, ReturnSuccess):
    """System command configuration requiring manual unlock."""

    _command_lock: bool = True


class CalibrateEncoder(SystemCommand):
    """Calibrate encoder command configuration."""

    _code: Code = Code.CAL_ENCODER
    _protocol: Protocol = Protocol.CAL_ENCODER


class ZeroAllPositions(SystemCommand):
    """Zero all positions command configuration."""

    _code: Code = Code.ZERO_ALL_POSITIONS
    _protocol: Protocol = Protocol.ZERO_ALL_POSITIONS


class ClearStall(SystemCommand):
    """Clear stall command configuration."""

    _code: Code = Code.CLEAR_STALL
    _protocol: Protocol = Protocol.CLEAR_STALL


class FactoryReset(SystemCommand):
    """Factory reset command configuration."""

    _code: Code = Code.FACTORY_RESET
    _protocol: Protocol = Protocol.FACTORY_RESET
