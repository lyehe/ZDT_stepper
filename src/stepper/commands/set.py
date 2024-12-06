"""Set commands for stepper motor."""

from stepper.commands.commands import (
    Command,
    ReturnSuccess,
    TakeStoreSetting,
    WithClassParams,
    WithEnumParams,
)
from stepper.stepper_core.configs import (
    Address,
    Code,
    Kpid,
    LoopMode,
    Microstep,
    OpenLoopCurrent,
    Protocol,
    SpeedReduction,
)
from stepper.stepper_core.parameters import ConfigParams, StartSpeedParams

__all__ = [
    "SetMicrostep",
    "SetID",
    "SetLoopMode",
    "SetOpenLoopCurrent",
    "SetPID",
    "SetStartSpeed",
    "SetReduction",
    "SetConfig",
]


class SetCommand(TakeStoreSetting, ReturnSuccess, Command):
    """Set command configuration."""


class SetMicrostep(WithClassParams, SetCommand):
    """Set microstep command configuration."""

    _code = Code.SET_MICROSTEP
    _protocol = Protocol.SET_MICROSTEP
    _command_lock: bool = True
    ParamsType = Microstep


class SetID(WithClassParams, SetCommand):
    """Set ID command configuration."""

    _code = Code.SET_ID
    _protocol = Protocol.SET_ID
    _command_lock: bool = True
    ParamsType = Address


class SetLoopMode(WithEnumParams, SetCommand):
    """Set loop mode command configuration."""

    _code = Code.SET_LOOP_MODE
    _protocol = Protocol.SET_LOOP_MODE
    _command_lock: bool = True
    ParamsType = LoopMode


class SetOpenLoopCurrent(WithClassParams, SetCommand):
    """Set open loop current command configuration."""

    _code = Code.SET_OPEN_LOOP_CURRENT
    _protocol = Protocol.SET_OPEN_LOOP_CURRENT
    _command_lock: bool = True
    ParamsType = OpenLoopCurrent


class SetPID(WithClassParams, SetCommand):
    """Set PID parameters command configuration."""

    _code = Code.SET_PID
    _protocol = Protocol.SET_PID
    _command_lock: bool = True
    ParamsType = Kpid


class SetStartSpeed(WithClassParams, SetCommand):
    """Set start speed command configuration."""

    _code = Code.SET_START_SPEED
    _protocol = Protocol.SET_START_SPEED
    _command_lock: bool = True
    ParamsType = StartSpeedParams


class SetReduction(WithClassParams, SetCommand):
    """Set speed reduction command configuration."""

    _code = Code.SET_REDUCTION
    _protocol = Protocol.SET_REDUCTION
    _command_lock: bool = True
    ParamsType = SpeedReduction


class SetConfig(WithClassParams, SetCommand):
    """Set configuration command configuration."""

    _code = Code.SET_CONFIG
    _protocol = Protocol.SET_CONFIG
    _command_lock: bool = True
    ParamsType = ConfigParams
