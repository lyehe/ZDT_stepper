"""Command classes to construct commands and handle responses."""

from abc import ABC, abstractmethod
from collections import OrderedDict
from dataclasses import dataclass, field
from logging import getLogger
from time import sleep, time

from serial import Serial
from stepper_constants import (
    Address,
    ChecksumMode,
    Code,
    StatusCode,
    SystemConstants,
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
    delay: float | None = field(default=None)

    _command: bytes = field(init=False)
    _response: bytes = field(init=False)
    _timestamp: float = field(init=False)

    def __post_init__(self):
        """Add checksum to the command."""
        self._command = _add_checksum(self._input, self.checksum_mode)
        self._timestamp = time()

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
        """Data length without address, code, and checksum."""
        return self._response_length - 3

    def _reset_buffers(self):
        """Reset input and output buffers."""
        self.serial_connection.reset_input_buffer()
        self.serial_connection.reset_output_buffer()

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

    def _read_checksum(self, response: bytes) -> int:
        """Read checksum from response."""
        checksum = self.serial_connection.read(1)
        expected_checksum = _add_checksum(response, self.checksum_mode)
        if checksum != expected_checksum:
            raise CommandError(f"Invalid checksum: {checksum}, expected: {expected_checksum}")
        return checksum

    def execute(self) -> bool:
        """Send the command to the serial port and read response."""
        self.serial_connection.reset_output_buffer()

        response = bytearray()
        error_count = 0

        while error_count < SystemConstants.MAX_RETRIES:
            try:
                self.serial_connection.write(self._command)
                address = self._read_address()
                code = self._read_code()
                data = self._read_data()
                self._process_data(data)

                response.extend(address, code.value, data)
                checksum = self._read_checksum(bytes(response))
                self._response = bytes(response + checksum)
                if self.delay:
                    sleep(self.delay)
                break
            except CommandError:
                self._reset_buffers()
                error_count += 1

        return error_count < SystemConstants.MAX_RETRIES


@dataclass
class BroadcastCommand(Command):
    """Base class for broadcast commands sent to all devices simultaneously.

    :param checksum_mode: Checksum calculation mode
    """

    addr: Address = field(default=Address.broadcast, init=False)
