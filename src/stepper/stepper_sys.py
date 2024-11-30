from dataclasses import dataclass
from logging import getLogger
from .stepper_constants import (
    Code,
    Protocol,
)
from .stepper_command import Command
from .stepper_exceptions import CommandError

logger = getLogger(__name__)


@dataclass
class CalibrateEncoder(Command):
    """Calibrate encoder command configuration"""

    confirm: bool = False

    @property
    def code(self) -> Code:
        return Code.CAL_ENCODER

    @property
    def command(self) -> bytes:
        if self.confirm:
            return bytes([self.addr, self.code, Protocol.CAL_ENCODER])
        else:
            logger.warning("Encoder calibration not confirmed")
            raise CommandError("Encoder calibration not confirmed")


@dataclass
class ClearPosition(Command):
    """Clear position command configuration"""

    confirm: bool = False

    @property
    def code(self) -> Code:
        return Code.CLEAR_POS

    @property
    def command(self) -> bytes:
        if self.confirm:
            return bytes([self.addr, self.code, Protocol.CLEAR_POS])
        else:
            logger.warning("Clear position not confirmed")
            raise CommandError("Clear position not confirmed")


@dataclass
class ClearStall(Command):
    """Clear stall command configuration"""

    confirm: bool = False

    @property
    def code(self) -> Code:
        return Code.CLEAR_STALL

    @property
    def command(self) -> bytes:
        if self.confirm:
            return bytes([self.addr, self.code, Protocol.CLEAR_STALL])
        else:
            logger.warning("Clear stall not confirmed")
            raise CommandError("Clear stall not confirmed")


@dataclass
class FactoryReset(Command):
    """Factory reset command configuration"""

    confirm: bool = False

    @property
    def code(self) -> Code:
        return Code.RESET

    @property
    def command(self) -> bytes:
        if self.confirm:
            return bytes([self.addr, self.code, Protocol.RESET])
        else:
            logger.warning("Factory reset not confirmed")
            raise CommandError("Factory reset not confirmed")
