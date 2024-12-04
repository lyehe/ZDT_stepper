"""Command classes to construct commands and handle responses."""

from abc import ABC, abstractmethod
from collections import OrderedDict
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

    serial_connection: Serial

    addr: Address = field(default=Address.default)
    checksum_mode: ChecksumMode = field(default=ChecksumMode.default)
    max_retries: int = field(default=5)
    delay: float | None = field(default=None)

    _command: bytes = field(init=False)
    _response: bytes = field(init=False)

    _read_timeout: float = field(init=False, default=0.005)

    def __post_init__(self):
        """Add checksum to the command."""
        self._command = _add_checksum(self.checksum_mode, self._input)

    @property
    @abstractmethod
    def _code(self) -> Code:
        """Command serial code."""
        ...

    @property
    @abstractmethod
    def _input(self) -> bytes:
        """Command bytes defined for each command."""
        ...

    @property
    @abstractmethod
    def _response_length(self) -> int:
        """Response length."""
        ...

    @property
    def _data_length(self) -> int:
        """Data length."""
        return self._response_length - 3

    def _read_address(self) -> Address:
        """Read address from response."""
        address = self.serial_connection.read(1)
        if self.addr == Address.broadcast:
            if address != 1:
                raise CommandError(f"Received address: {address}, expected: 1")
        elif address != self.addr:
            raise CommandError(f"Received address: {address}, expected: {self.addr}")
        return Address(address)
    
    def _read_code(self) -> Code:
        """Read code from response."""
        code = self.serial_connection.read(1)
        if code == StatusCode.ERROR:
            raise CommandError(f"Error code: {code}")
        return Code(code)
    
    def _read_data(self) -> bytes:
        """Read data from response."""
        data_length = self._data_length
        data = self.serial_connection.read(data_length)
        return data
    
    @abstractmethod
    def _process_data(self, data: bytes) -> None:
        """Process data from response."""
        ...
    
    def _read_checksum(self) -> int:
        """Read checksum from response."""
        checksum = self.serial_connection.read(1)
        expected_checksum = _add_checksum(self._response, self.checksum_mode)
        if checksum != expected_checksum:
            raise CommandError(f"Invalid checksum: {checksum}, expected: {expected_checksum}")
        return checksum


    def execute(self):
        """Send the command to the serial port and read response."""
        self.serial_connection.write(self._command)

        response = bytearray()
        timeout_count = 0

        address = self._read_address()
        code = self._read_code()
        data = self._read_data()
        self._process_data(data)
        self._response = bytes(response)
        checksum = self._read_checksum()

        while len(response) < 4:  # Minimum response length



        while len(response) < 4:  # Minimum response length


        while len(response) < 4:  # Minimum response length
            self.serial_connection.timeout = self.read_timeout  # Set timeout for each attempt
            if self.serial_connection.in_waiting:
                response.extend(self.serial_connection.read(self.serial_connection.in_waiting))
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
        if self.serial_connection.in_waiting:
            response.extend(self.serial_connection.read(self.serial_connection.in_waiting))

        self.response = bytes(response)
        if self.delay:
            sleep(self.delay)


@dataclass
class BroadcastCommand(Command, ABC):
    """Base class for broadcast commands sent to all devices simultaneously.

    :param checksum_mode: Checksum calculation mode
    """

    addr: Address = field(default=Address.broadcast, init=False)
