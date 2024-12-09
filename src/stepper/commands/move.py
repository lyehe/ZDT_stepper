"""Move commands for stepper motor."""

from stepper.commands.commands import (
    ReturnSuccess,
    TakeNoSetting,
    TakeSyncSetting,
    WithClassParams,
    WithEnumParams,
    WithNoParams,
)
from stepper.stepper_core.configs import (
    Code,
    EnableFlag,
    Protocol,
)
from stepper.stepper_core.parameters import JogParams, PositionParams

__all__ = ["Enable", "Jog", "Move", "EStop", "SyncMove"]


class MoveCommand(TakeSyncSetting, ReturnSuccess):
    """Move command configuration. All move commands inherit from this class."""


class Enable(MoveCommand, WithEnumParams):
    """Enable command."""

    _code: Code = Code.ENABLE
    _protocol: Protocol = Protocol.ENABLE
    ParamsType = EnableFlag


class Disable(Enable):
    """Disable command, releases the motor from the enable state."""

    def _process_params(self, params: EnableFlag | None) -> EnableFlag:
        return EnableFlag.DISABLE


class Jog(MoveCommand, WithClassParams):
    """Jog command configuration."""

    _code: Code = Code.JOG
    ParamsType = JogParams


class Move(MoveCommand, WithClassParams):
    """Move command configuration."""

    _code: Code = Code.MOVE
    ParamsType = PositionParams


class EStop(MoveCommand, WithNoParams):
    """Emergency stop command configuration."""

    _code: Code = Code.ESTOP
    _protocol: Protocol = Protocol.ESTOP


class SyncMove(WithNoParams, TakeNoSetting, ReturnSuccess):
    """Sync move command configuration."""

    _code: Code = Code.SYNC_MOVE
    _protocol: Protocol = Protocol.SYNC_MOVE
