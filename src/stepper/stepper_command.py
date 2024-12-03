"""Command classes to construct commands and handle responses."""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from logging import getLogger
from time import sleep

from serial import Serial
from stepper_constants import (
    Address,
    ChecksumMode,
    Code,
    StatusCode,
)
from stepper_exceptions import CommandError

logger = getLogger(__name__)


def _add_checksum(command_bytes: bytes, checksum_mode: ChecksumMode) -> bytes:
    """Add checksum to the command."""

    def _calculate_checksum(command_bytes: bytes, checksum_mode: ChecksumMode) -> int:
        """Calculate checksum based on selected method."""

        def _calculate_xor_checksum(data: bytes) -> int:
            """Calculate XOR checksum of bytes."""
            checksum = 0
            for byte in data:
                checksum ^= byte
            return checksum

        def _calculate_crc8(data: bytes) -> int:
            """Calculate CRC-8 with polynomial x^8 + x^2 + x + 1 (0x07)."""
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
                return StatusCode.FIXED_CHECKSUM_BYTE
            case ChecksumMode.XOR:
                return _calculate_xor_checksum(command_bytes)
            case ChecksumMode.CRC8:
                return _calculate_crc8(command_bytes)
            case _:
                raise CommandError("Invalid checksum mode")

    checksum = _calculate_checksum(command_bytes, checksum_mode)
    return command_bytes + bytes([checksum])


def _int(input: bytes) -> int:
    return int.from_bytes(input, "big")


def _signed_int(input: bytes) -> int:
    sign = -1 if input[0] == 1 else 1
    return sign * _int(input[1:])


@dataclass
class Command(ABC):
    """Command configuration class.

    :param addr: Device address
    :param checksum_mode: Checksum calculation mode
    :param max_retries: Maximum number of retries
    :param read_timeout: Read timeout
    :param delay: Delay after sending the command
    """

    addr: Address = field(default=Address.default)
    checksum_mode: ChecksumMode = field(default=ChecksumMode.default)
    max_retries: int = field(default=5)
    delay: float | None = field(default=None)

    _command: bytes = field(init=False)
    _response: bytes = field(init=False)

    _read_timeout: float = field(init=False, default=0.005)

    def __post_init__(self):
        """Add checksum to the command."""
        self._command = _add_checksum(self.checksum_mode, self._command_bytes)

    @property
    @abstractmethod
    def _code(self) -> Code:
        """Command serial code."""
        ...

    @property
    @abstractmethod
    def _command_bytes(self) -> bytes:
        """Command bytes defined for each command."""
        ...

    @property
    def response(self) -> bytes:
        """Response bytes."""
        return self._response

    @response.setter
    def response(self, response: bytes):
        """Set response bytes."""
        self._response = response

    @property
    @abstractmethod
    def _response_length(self) -> int:
        """Response length."""
        return 4

    @property
    def response_dict(self) -> dict[str, int]:
        """Response dictionary."""
        return {
            "addr": Address(self._response[0]),
            "code": Code(self._response[1]),
            "status": self._response[2],
            "checksum": self._response[3],
        }

    def execute(self, serial_connection: Serial):
        """Send the command to the serial port and read response."""
        serial_connection.write(self._command)

        response = bytearray()
        timeout_count = 0

        while len(response) < 4:  # Minimum response length
            serial_connection.timeout = self.read_timeout  # Set timeout for each attempt
            if serial_connection.in_waiting:
                response.extend(serial_connection.read(serial_connection.in_waiting))
                timeout_count = 0
            else:
                timeout_count += 1
                if timeout_count >= self.max_retries:
                    raise CommandError(
                        f"Timeout waiting for response after "
                        f"{self.max_retries * self.read_timeout * 1000:.1f}ms"
                    )
                sleep(self.read_timeout)  # Wait for the timeout period before retrying

        # Read any remaining data (if any)
        if serial_connection.in_waiting:
            response.extend(serial_connection.read(serial_connection.in_waiting))

        self.response = bytes(response)
        if self.delay:
            sleep(self.delay)


@dataclass
class BroadcastCommand(Command, ABC):
    """Base class for broadcast commands sent to all devices simultaneously.

    :param checksum_mode: Checksum calculation mode
    """

    addr: Address = field(default=Address.broadcast, init=False)
