from dataclasses import dataclass, field
from logging import getLogger

from stepper_constants import (
    AbsoluteFlag,
    Acceleration,
    Code,
    Direction,
    EnableFlag,
    Protocol,
    PulseCount,
    Speed,
    SyncFlag,
)

from .stepper_command import BroadcastCommand, Command, _add_checksum

logger = getLogger(__name__)


@dataclass
class MoveCommand(Command):
    """Move command configuration"""

    sync: SyncFlag = SyncFlag.default


@dataclass
class Enable(MoveCommand):
    """Enable command

    :param enable_status: Enable status flag
    """

    enable_status: EnableFlag = EnableFlag.ENABLE

    @property
    def _code(self) -> Code:
        return Code.ENABLE

    @property
    def _command_bytes(self) -> bytes:
        return bytes([self.addr, self._code, Protocol.ENABLE, self.enable_status, self.sync])


@dataclass
class Disable(MoveCommand):
    """Disable command, releases the motor from the enable state"""

    @property
    def _code(self) -> Code:
        return Code.ENABLE

    @property
    def _command_bytes(self) -> bytes:
        return bytes([self.addr, self._code, Protocol.ENABLE, EnableFlag.DISABLE, self.sync])


@dataclass
class Jog(MoveCommand):
    """Jog command configuration

    :param direction: Movement direction
    :param speed: Movement speed
    :param acceleration: Movement acceleration
    :param sync: Sync flag
    """

    direction: Direction = Direction.default
    speed: Speed = Speed.default
    acceleration: Acceleration = Acceleration.default

    @property
    def _code(self) -> Code:
        return Code.JOG

    @property
    def _command_bytes(self) -> bytes:
        return bytes(
            [
                self.addr,
                self._code,
                self.direction,
                *self.speed.bytes,
                self.acceleration,
                self.sync,
            ]
        )


@dataclass
class JogCW(Jog):
    """Jog in the clockwise direction"""

    direction: Direction = field(default=Direction.CW, init=False)


@dataclass
class JogCCW(Jog):
    """Jog in the counterclockwise direction"""

    direction: Direction = field(default=Direction.CCW, init=False)


@dataclass
class JogStop(Jog):
    """Stops the jog movement"""

    speed: Speed = field(default=Speed.stop, init=False)


@dataclass
class JogStopAll(Jog, BroadcastCommand):
    """Stops all jog movements across all devices"""

    speed: Speed = field(default=Speed.stop, init=False)


@dataclass
class Move(MoveCommand):
    """Move command configuration

    :param direction: Movement direction
    :param speed: Movement speed
    :param acceleration: Movement acceleration
    :param pulse_count: Number of pulses to move
    :param mode: Relative/Absolute mode
    :param sync: Sync flag
    """

    direction: Direction = Direction.default
    speed: Speed = Speed.default
    acceleration: Acceleration = Acceleration.default
    pulse_count: PulseCount = PulseCount.default
    mode: AbsoluteFlag = AbsoluteFlag.default

    @property
    def _code(self) -> Code:
        return Code.MOVE

    @property
    def _command_bytes(self) -> bytes:
        return bytes(
            [
                self.addr,
                self._code,
                self.direction,
                *self.speed.bytes,
                self.acceleration,
                *self.pulse_count.bytes,
                self.mode,
                self.sync,
            ]
        )


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
        self.send_command = _add_checksum(self.checksum_mode, self._command_bytes)


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
    def _command_bytes(self) -> bytes:
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
    def _command_bytes(self) -> bytes:
        return bytes([self.addr, self._code, Protocol.SYNC_MOVE])
