"""Command classes to construct commands and handle responses."""

from abc import ABC, abstractmethod
from logging import getLogger
from time import sleep, time
from typing import TypeAlias, TypeVar

from .stepper_constants import (
    Address,
    ChecksumMode,
    Code,
    Protocol,
    StatusCode,
    StoreFlag,
    SyncFlag,
    SystemConstants,
)
from .stepper_exceptions import CommandError
from .stepper_parameters import DeviceParams

logger = getLogger(__name__)


ParamsType = TypeVar("ParamsType")
ValueType = TypeVar("ValueType", int, bytes, float)
GroupSettingType: TypeAlias = StoreFlag | SyncFlag


def _calculate_checksum(command_bytes: bytes, checksum_mode: ChecksumMode) -> bytes:
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
            checksum = StatusCode.FIXED_CHECKSUM_BYTE
        case ChecksumMode.XOR:
            checksum = _calculate_xor_checksum(command_bytes)
        case ChecksumMode.CRC8:
            checksum = _calculate_crc8(command_bytes)
        case _:
            raise CommandError("Invalid checksum mode")

    return bytes([checksum])


def _add_checksum(command_bytes: bytes, checksum_mode: ChecksumMode) -> bytes:
    """Add checksum to the command."""
    return command_bytes + _calculate_checksum(command_bytes, checksum_mode)


def _int(input: bytes) -> int:
    return int.from_bytes(input, "big")


def _signed_int(input: bytes) -> int:
    sign = -1 if input[0] == 1 else 1
    return sign * _int(input[1:])


class Command(ABC):
    """Command configuration class."""

    _code: Code
    _response_length: int
    _protocol: Protocol | None = None

    def __init__(
        self,
        device: DeviceParams,
        params: ParamsType | None = None,
        setting: GroupSettingType | None = None,
    ):
        """Initialize the command.

        The command is executed immediately after initialization.
        :param serial_connection: Serial connection
        :param address: Device address
        :param checksum_mode: Checksum calculation mode
        :param delay: Delay after sending the command
        """
        self._timestamp = time()

        self.address = device.address
        self.checksum_mode = device.checksum_mode
        self.delay = device.delay
        self.params = self._process_params(params)
        self.setting = self._process_setting(setting)

        self._command = _add_checksum(self._command_body, self.checksum_mode)
        self._response: ValueType = None
        self._raw_value: ValueType = None
        self._value: ValueType = None
        self.serial_connection = device.serial_connection
        self._connection_flag = self.serial_connection.is_open

        if not self._connection_flag:
            logger.debug(f"Opening {self.serial_connection.name}")
            with self.serial_connection:
                self._success = self._execute()
        else:
            self._success = self._execute()

    @abstractmethod
    def _process_params(self, params: ParamsType | None) -> ParamsType:
        """Process parameters."""
        ...

    @abstractmethod
    def _process_setting(self, setting: GroupSettingType) -> GroupSettingType:
        """Process setting."""
        ...

    @abstractmethod
    def _process_data(self, data: bytes) -> bool:
        """Process data from response."""
        ...

    @property
    def _command_body(self) -> bytes:
        """Command bytes defined for each command."""
        body = bytes([self.address, self._code])
        if self._protocol is not None:
            body += self._protocol.bytes
        if self.params is not None:
            body += self.params.bytes
        if self.setting is not None:
            body += self.setting.bytes
        return body

    @property
    def initialization_time(self) -> float:
        """Initial timestamp."""
        return self._timestamp

    @property
    def _data_length(self) -> int:
        """Data length without address, code, and checksum."""
        return self._response_length - 3

    @property
    def response(self) -> bytes:
        """Command response."""
        return self._response

    @property
    def raw_value(self) -> bytes:
        """Raw command value."""
        return self._response[2:-1]

    @property
    def value(self) -> ValueType:
        """Command value."""
        return self._value

    @property
    def is_serial_active(self) -> bool:
        """Serial connection active."""
        return self._connection_flag

    @property
    def is_success(self) -> bool:
        """Command success."""
        return self._success

    def _reset_buffers(self):
        """Reset input and output buffers."""
        logger.debug("Resetting buffers")
        self.serial_connection.reset_input_buffer()
        self.serial_connection.reset_output_buffer()

    def _read_address(self) -> bytes:
        """Read address from response and validate."""
        address = self.serial_connection.read(1)
        if self.address == Address.broadcast:
            if address != bytes([1]):
                raise CommandError(f"Received address: {address}, expected: 1")
        elif address != self.address.bytes:
            raise CommandError(f"Received address: {address}, expected: {self.address}")
        logger.debug(f"Received address: {address}, expected: {self.address}")
        return address

    def _read_code(self) -> bytes:
        """Read code from response and validate."""
        code = self.serial_connection.read(1)
        if code == StatusCode.ERROR.bytes:
            raise CommandError(f"Error code: {code}")
        logger.debug(f"Received code: {code}")
        return code

    def _read_data(self) -> bytes:
        """Read data from response."""
        data = self.serial_connection.read(self._data_length)
        logger.debug(f"Received data: {data}")
        return data

    def _read_checksum(self, response: bytes) -> bytes:
        """Read checksum from response and validate."""
        checksum = self.serial_connection.read(1)
        expected_checksum = _calculate_checksum(response, self.checksum_mode)
        if checksum != expected_checksum:
            raise CommandError(f"Invalid checksum: {checksum}, expected: {expected_checksum}")
        logger.debug(f"Received checksum: {checksum}, expected: {expected_checksum}")
        return checksum

    def _delay(self) -> None:
        """Delay after sending the command."""
        if self.delay:
            logger.debug(f"Delaying for {self.delay} seconds")
            sleep(self.delay)

    def _execute(self) -> bool:
        """Send the command to the serial port and read response.

        :return: True if the command was successful, False otherwise
        """
        self.serial_connection.reset_output_buffer()
        tries = 0

        while tries < SystemConstants.MAX_RETRIES:
            try:
                self.serial_connection.write(self._command)
                logger.debug(f"Sending {self._command}")
                address = self._read_address()
                code = self._read_code()
                data = self._read_data()
                success = self._process_data(data)
                checksum = self._read_checksum(address + code + data)
                self._response = address + code + data + checksum
                self._delay()
                break
            except CommandError:
                self._reset_buffers()
            finally:
                tries += 1

        return tries < SystemConstants.MAX_RETRIES and success

    def __repr__(self) -> str:
        """Representation of the command."""
        return f"{self.__class__.__name__}({self.address}, {self.params}, {self.setting})"

    def __str__(self) -> str:
        """String representation of the command."""
        return f"{self.__class__.__name__}({self.address}, {self.params}, {self.setting})"
