"""Set commands for stepper motor."""

from src.stepper.stepper_parameters import StartSpeedParams

from .stepper_command import (
    Command,
    ReturnSuccess,
    TakeStoreSetting,
    WithClassParams,
    WithEnumParams,
)
from .stepper_constants import (
    Address,
    Code,
    Kpid,
    LoopMode,
    Microstep,
    OpenLoopCurrent,
    Protocol,
    SpeedReduction,
)


class SetCommand(TakeStoreSetting, ReturnSuccess, Command):
    """Set command configuration."""


class SetMicrostep(WithClassParams, SetCommand):
    """Set microstep command configuration."""

    _code = Code.SET_MICROSTEP
    _protocol = Protocol.SET_MICROSTEP
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
    ParamsType = LoopMode


class SetOpenLoopCurrent(WithClassParams, SetCommand):
    """Set open loop current command configuration."""

    _code = Code.SET_OPEN_LOOP_CURRENT
    _protocol = Protocol.SET_OPEN_LOOP_CURRENT
    ParamsType = OpenLoopCurrent


class SetPID(WithClassParams, SetCommand):
    """Set PID parameters command configuration."""

    _code = Code.SET_PID
    _protocol = Protocol.SET_PID
    ParamsType = Kpid


class SetStartSpeed(WithClassParams, SetCommand):
    """Set start speed command configuration."""

    _code = Code.SET_START_SPEED
    _protocol = Protocol.SET_START_SPEED
    ParamsType = StartSpeedParams


class SetReduction(WithClassParams, SetCommand):
    """Set speed reduction command configuration."""

    _code = Code.SET_REDUCTION
    _protocol = Protocol.SET_REDUCTION
    ParamsType = SpeedReduction
