"""Move commands for stepper motor."""

from dataclasses import dataclass
from logging import getLogger

from .stepper_command import Command
from .stepper_constants import (
    Code,
    EnableFlag,
    Protocol,
    StatusCode,
    SyncFlag,
)
from .stepper_exceptions import CommandError
from .stepper_parameters import JogParams, PositionParams

logger = getLogger(__name__)


class MoveCommand(Command):
    """Move command configuration. All move commands inherit from this class."""

    _response_length: int = 4

    def _process_setting(self, setting: SyncFlag | None) -> SyncFlag:
        """Validate setting."""
        if setting is None:
            logger.warning("Sync flag not provided, using default")
            setting = SyncFlag.NO_SYNC
        return setting

    def _process_data(self, data: bytes) -> bool:
        """The data is the status code."""
        if data == StatusCode.CONDITIONAL_ERROR.bytes:
            raise CommandError("Move command failed due to condition error.")
        return data == StatusCode.SUCCESS.bytes


class Enable(MoveCommand):
    """Enable command."""

    _code: Code = Code.ENABLE
    _protocol: Protocol = Protocol.ENABLE

    def _process_params(self, params: EnableFlag | None) -> EnableFlag:
        """The parameters are always ENABLE."""
        return EnableFlag.ENABLE


class Disable(Enable):
    """Disable command, releases the motor from the enable state."""

    def _process_params(self, params: EnableFlag | None) -> EnableFlag:
        """The parameters are always DISABLE."""
        return EnableFlag.DISABLE


class Jog(MoveCommand):
    """Jog command configuration.

    :param velocity_data: Configuration for movement velocity, including speed and acceleration
    """

    _code: Code = Code.JOG

    def _process_params(self, params: JogParams) -> JogParams:
        return params


class Move(MoveCommand):
    """Move command configuration.

    :param direction: Movement direction
    :param speed: Movement speed
    :param acceleration: Movement acceleration
    :param pulse_count: Number of pulses to move
    :param mode: Relative/Absolute mode
    :param sync: Sync flag
    """

    _code: Code = Code.MOVE

    def _process_params(self, params: PositionParams) -> PositionParams:
        """The parameters are always position data."""
        return params


class EStop(MoveCommand):
    """Emergency stop command configuration."""

    _code: Code = Code.ESTOP
    _protocol: Protocol = Protocol.ESTOP

    def _process_params(self, params: None) -> None:
        """The parameters are always None."""
        return None


class SyncMove(Command):
    """Sync move command configuration."""

    _code: Code = Code.SYNC_MOVE
    _protocol: Protocol = Protocol.SYNC_MOVE

    def _process_params(self, params: None) -> None:
        """The parameters are always None."""
        return None
