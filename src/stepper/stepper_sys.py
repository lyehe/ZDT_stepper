from dataclasses import dataclass
from logging import getLogger

from .stepper_command import Command
from .stepper_constants import (
    Code,
    Protocol,
)
from .stepper_exceptions import CommandError

logger = getLogger(__name__)


@dataclass
class SystemCommand(Command):
    """System command configuration requiring confirmation"""

    confirm: bool = False

    def _check_confirm(self, command: bytes) -> bytes:
        if self.confirm:
            return command
        else:
            logger.warning(f"{self._code.name} not confirmed. Please set confirm to True.")
            raise CommandError(f"{self._code.name} not confirmed")


@dataclass
class CalibrateEncoder(SystemCommand):
    """Calibrate encoder command configuration"""

    @property
    def _code(self) -> Code:
        return Code.CAL_ENCODER

    @property
    def _input(self) -> bytes:
        return self._check_confirm(bytes([self.addr, self._code, Protocol.CAL_ENCODER]))


@dataclass
class ZeroAllPositions(SystemCommand):
    """Zero all positions command configuration"""

    @property
    def _code(self) -> Code:
        return Code.ZERO_ALL_POSITIONS

    @property
    def _input(self) -> bytes:
        return self._check_confirm(bytes([self.addr, self._code, Protocol.ZERO_ALL_POSITIONS]))


@dataclass
class ClearStall(SystemCommand):
    """Clear stall command configuration"""

    @property
    def _code(self) -> Code:
        return Code.CLEAR_STALL

    @property
    def _input(self) -> bytes:
        return self._check_confirm(bytes([self.addr, self._code, Protocol.CLEAR_STALL]))


@dataclass
class FactoryReset(SystemCommand):
    """Factory reset command configuration"""

    @property
    def _code(self) -> Code:
        return Code.FACTORY_RESET

    @property
    def _input(self) -> bytes:
        return self._check_confirm(bytes([self.addr, self._code, Protocol.FACTORY_RESET]))
