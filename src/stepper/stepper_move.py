"""Move commands for stepper motor."""

from dataclasses import dataclass, field
from logging import getLogger

from .stepper_command import Command, _add_checksum
from .stepper_constants import (
    AbsoluteFlag,
    Code,
    Direction,
    EnableFlag,
    Protocol,
    PulseCount,
    StatusCode,
    SyncFlag,
)
from .stepper_exceptions import CommandError
from .stepper_parameters import PositionParams, VelocityParams

logger = getLogger(__name__)


class MoveCommand(Command):
    """Move command configuration. All move commands inherit from this class.

    :param sync: Sync flag

    """

    _response_length: int = 4

    def _validate_setting(self, setting: SyncFlag | None) -> SyncFlag:
        """Validate setting."""
        if setting is None:
            logger.warning("Sync flag not provided, using default")
            setting = SyncFlag.default
        return setting

    def _process_data(self, data: bytes) -> bool:
        """Process data from response."""
        if data == StatusCode.SUCCESS.bytes:
            return True
        raise CommandError(f"Move command failed with status code: {data}")


class Enable(MoveCommand):
    """Enable command.

    Usage example: Enable(addr=0x01, sync=SyncFlag.ENABLE)
    :param enable_status: Enable status flag
    """

    _code: Code = Code.ENABLE

    def _validate_params(self, enable_status: EnableFlag | None) -> EnableFlag:
        """Validate parameters."""
        if enable_status is None:
            enable_status = EnableFlag.ENABLE
        return enable_status

    @property
    def _command_body(self) -> bytes:
        return bytes([self.addr, self._code, Protocol.ENABLE, self.params, self.setting])


class Disable(Enable):
    """Disable command, releases the motor from the enable state."""

    def _validate_params(self, enable_status: EnableFlag | None) -> EnableFlag:
        """Validate parameters."""
        if enable_status is None:
            enable_status = EnableFlag.DISABLE
        return enable_status


class Jog(MoveCommand):
    """Jog command configuration.

    :param velocity_data: Configuration for movement velocity, including speed and acceleration
    :param sync: Sync flag (inherited from MoveCommand)
    """

    _code: Code = Code.JOG

    @property
    def _command_body(self) -> bytes:
        if isinstance(self.params, VelocityParams):
            return bytes(
                [
                    self.addr,
                    self._code,
                    self.params.bytes,
                    self.setting,
                ]
            )
        raise ValueError("Invalid params type")

    def _process_data(self, data: bytes) -> None:
        self.success = data == StatusCode.SUCCESS
        if not self.success:
            raise CommandError(f"Jog command failed with status code: {data}")


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

    @property
    def _command_body(self) -> bytes:
        if isinstance(self.params, PositionParams):
            return bytes(
                [
                    self.addr,
                    self._code,
                    self.params.bytes,
                    self.setting,
                ]
            )
        raise ValueError("Invalid params type")


@dataclass
class GoTo(Move):
    """Go to command configuration"""

    mode: AbsoluteFlag = field(default=AbsoluteFlag.ABSOLUTE, init=False)


@dataclass
class MoveDeg(Move):
    """Move command configuration in degrees

    :param degrees: Number of degrees to move
    """

    degrees: float
    microstep_per_degree: float
    mode: AbsoluteFlag = field(default=AbsoluteFlag.RELATIVE, init=False)

    def __post_init__(self):
        if self.degrees < 0:
            self.direction = Direction.CCW
        else:
            self.direction = Direction.CW
        self.pulse_count = PulseCount(self.degrees * self.microstep_per_degree)
        self.send_command = _add_checksum(self.checksum_mode, self._command_body)


@dataclass
class GotoDeg(MoveDeg):
    """Go to command configuration in degrees"""

    mode: AbsoluteFlag = field(default=AbsoluteFlag.ABSOLUTE, init=False)


@dataclass
class EStop(MoveCommand):
    """Emergency stop command configuration

    :param sync: Sync flag
    """

    @property
    def _code(self) -> Code:
        return Code.ESTOP

    @property
    def _command_body(self) -> bytes:
        return bytes([self.addr, self._code, Protocol.ESTOP, self.sync])


@dataclass
class EStopAll(EStop, BroadcastCommand):
    """Emergency stop command configuration for all devices"""


@dataclass
class SyncMove(Command):
    """Sync move command configuration"""

    @property
    def _code(self) -> Code:
        return Code.SYNC_MOVE

    @property
    def _command_body(self) -> bytes:
        return bytes([self.addr, self._code, Protocol.SYNC_MOVE])
