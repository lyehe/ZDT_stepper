from dataclasses import dataclass
from logging import getLogger
from .stepper_command import Command
from stepper_constants import (
    Code,
    Protocol,
    EnableFlag,
    Direction,
    SyncFlag,
    AbsoluteFlag,
    Speed,
    Acceleration,
    PulseCount,
)

logger = getLogger(__name__)


@dataclass
class Enable(Command):
    """Enable command configuration

    :param enable_status: Enable status flag
    :param sync: Sync flag
    """

    enable_status: EnableFlag
    sync: SyncFlag

    @property
    def code(self) -> Code:
        return Code.ENABLE

    @property
    def command(self) -> bytes:
        return bytes(
            [self.addr, self.code, Protocol.ENABLE, self.enable_status, self.sync]
        )


@dataclass
class Jog(Command):
    """Jog command configuration

    :param direction: Movement direction
    :param speed: Movement speed
    :param acceleration: Movement acceleration
    :param sync: Sync flag
    """

    direction: Direction
    speed: Speed
    acceleration: Acceleration
    sync: SyncFlag

    @property
    def code(self) -> Code:
        return Code.JOG

    @property
    def command(self) -> bytes:
        return bytes(
            [
                self.addr,
                self.code,
                self.direction,
                *self.speed.bytes,
                self.acceleration,
                self.sync,
            ]
        )


@dataclass
class Move(Command):
    """Move command configuration

    :param direction: Movement direction
    :param speed: Movement speed
    :param acceleration: Movement acceleration
    :param pulse_count: Number of pulses to move
    :param mode: Relative/Absolute mode
    :param sync: Sync flag
    """

    direction: Direction
    speed: Speed
    acceleration: Acceleration
    pulse_count: PulseCount
    mode: AbsoluteFlag
    sync: SyncFlag

    @property
    def code(self) -> Code:
        return Code.MOVE

    @property
    def command(self) -> bytes:
        return bytes(
            [
                self.addr,
                self.code,
                self.direction,
                *self.speed.bytes,
                self.acceleration,
                *self.pulse_count.bytes,
                self.mode,
                self.sync,
            ]
        )


@dataclass
class EStop(Command):
    """Emergency stop command configuration

    :param sync: Sync flag
    """

    sync: SyncFlag

    @property
    def code(self) -> Code:
        return Code.ESTOP

    @property
    def command(self) -> bytes:
        return bytes([self.addr, self.code, Protocol.ESTOP, self.sync])


@dataclass
class SyncMove(Command):
    """Sync move command configuration"""

    @property
    def code(self) -> Code:
        return Code.SYNC_MOVE

    @property
    def command(self) -> bytes:
        return bytes([self.addr, self.code, Protocol.SYNC_MOVE])
