from dataclasses import dataclass, field
from abc import ABC, abstractmethod
from logging import getLogger
from typing import Final
from stepper_constants import (
    Code,
    ChecksumMode,
    Address,
    StatusCode,
)
from stepper_exceptions import CommandError

logger = getLogger(__name__)


def _calculate_checksum(command_bytes: bytes, checksum_mode: ChecksumMode) -> int:
    """Calculate checksum based on selected method"""
    FIXED_CHECKSUM_BYTE: Final[int] = 0x6B

    def _calculate_xor_checksum(data: bytes) -> int:
        """Calculate XOR checksum of bytes"""
        checksum = 0
        for byte in data:
            checksum ^= byte
        return checksum

    def _calculate_crc8(data: bytes) -> int:
        """Calculate CRC-8 with polynomial x^8 + x^2 + x + 1 (0x07)"""
        crc = 0
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = (crc << 1) ^ 0x07
                else:
                    crc <<= 1
            crc &= 0xFF
        return crc

    match checksum_mode:
        case ChecksumMode.FIXED:
            return FIXED_CHECKSUM_BYTE
        case ChecksumMode.XOR:
            return _calculate_xor_checksum(command_bytes)
        case ChecksumMode.CRC8:
            return _calculate_crc8(command_bytes)
        case _:
            raise CommandError("Invalid checksum mode")


def _add_checksum(command_bytes: bytes, checksum_mode: ChecksumMode) -> bytes:
    checksum = _calculate_checksum(command_bytes, checksum_mode)
    return command_bytes + bytes([checksum])


def _int(input: bytes) -> int:
    return int.from_bytes(input, "big")


def _signed_int(input: bytes) -> int:
    sign = -1 if input[0] == 1 else 1
    return sign * _int(input[1:])


@dataclass
class Command(ABC):
    """Command configuration

    :param addr: Device address
    :param checksum_mode: Checksum calculation mode
    :param command: Command bytes (auto-generated)
    """

    addr: Address = field(default=Address.default, init=False)
    checksum_mode: ChecksumMode = field(default=ChecksumMode.default, init=False)
    send_command: bytes = field(init=False)
    _response: bytes = field(init=False)

    def __post_init__(self):
        self.send_command = _add_checksum(self.checksum_mode, self.command)

    @property
    @abstractmethod
    def code(self) -> Code: ...

    @property
    @abstractmethod
    def command(self) -> bytes: ...

    @property
    def response(self) -> bytes:
        return self._response

    @response.setter
    def response(self, response: bytes):
        self._response = response

    @property
    def response_dict(self) -> dict[str, int]:
        return {
            "addr": Address(self._response[0]),
            "code": Code(self._response[1]),
            "status": self._response[2],
            "checksum": self._response[3],
        }

    def verify(self):
        """Output result of the command"""
        match self.response_dict:
            case {"code": Code.ERROR, "status": StatusCode.ERROR}:
                logger.error(f"Device {self.addr}: {self.code.name} failed")
                raise CommandError("Error")
            case {"status": StatusCode.CONDITIONAL_ERROR}:
                logger.error(
                    f"Device {self.addr}: {self.code.name} failed due to conditional error"
                )
                raise CommandError("Conditional error")
            case {"status": StatusCode.SUCCESS}:
                logger.info(f"Device {self.addr}: {self.code.name} successful")
                return self.response_dict
            case _:
                raise CommandError("Invalid response")


@dataclass
class BroadcastCommand(Command, ABC):
    """Base class for broadcast commands sent to all devices simultaneously

    :param checksum_mode: Checksum calculation mode
    """

    addr: Address = field(default=Address.broadcast, init=False)
