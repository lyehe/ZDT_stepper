"""Command classes to construct commands and handle responses."""

from abc import ABC, abstractmethod
from logging import getLogger
from time import sleep, time
from typing import TypeAlias, TypeVar

from stepper.stepper_core.configs import (
    Address,
    ChecksumMode,
    Code,
    ExtendedIntEnum,
    Protocol,
    RangedInt,
    StatusCode,
    StoreFlag,
    SyncFlag,
    SystemConstants,
)

from ..stepper_core.exceptions import CommandError
from ..stepper_core.parameters import DeviceParams, StepperInput, StepperOutput

logger = getLogger(__name__)


ResponseType: TypeAlias = bytes | None
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


class Command(ABC):
    """Command configuration class."""

    _code: Code
    _response_length: int
    _protocol: Protocol | None = None
    _command_lock: bool = False

    ParamsType = TypeVar("ParamsType", bound=StepperInput | ExtendedIntEnum | RangedInt)
    DataType = TypeVar("DataType")
    ReturnType = TypeVar("ReturnType")

    def __init__(
        self,
        device: DeviceParams,
        params: ParamsType | None = None,
        setting: GroupSettingType | None = None,
    ):
        """Initialize the command.

        :param device: Device parameters - SerialConnection, Address, ChecksumMode, Delay
        :param params: Parameters - command specific parameters
        :param setting: Setting - StoreFlag or SyncFlag
        """
        if self._command_lock:
            raise CommandError("Command is locked.")

        self._timestamp = time()
        self._response: ResponseType = None
        self._raw_data: self.ReturnType = None
        self._data: self.DataType = None

        self.address = device.address
        self.checksum_mode = device.checksum_mode
        self.delay = device.delay
        self.params = self._process_params(params)
        self.setting = self._process_setting(setting)
        self._command = _add_checksum(self._command_body, self.checksum_mode)
        self.serial_connection = device.serial_connection

        if not self.serial_connection.is_open:
            logger.debug(f"Opening {self.serial_connection.name}")
            with self.serial_connection:
                self._status = self._execute()
        else:
            self._status = self._execute()

    @abstractmethod
    def _process_params(self, params: ParamsType | None) -> ParamsType:
        """Process parameters."""
        ...

    @abstractmethod
    def _process_setting(self, setting: GroupSettingType) -> GroupSettingType:
        """Process setting."""
        ...

    @abstractmethod
    def _process_data(self, data: bytes) -> StatusCode:
        """Process data from response."""
        ...

    @property
    def _command_body(self) -> bytes:
        """Command bytes defined for each command."""
        body = bytes([self.address, self._code])
        if self._protocol is not None:
            body += self._protocol.bytes
        if isinstance(self.setting, StoreFlag):
            body += self.setting.bytes
        if self.params is not None:
            body += self.params.bytes
        if isinstance(self.setting, SyncFlag):
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
    def raw_data(self) -> ParamsType:
        """Raw command data."""
        return self._raw_data

    @property
    def data(self) -> DataType:
        """Command data."""
        return self._data

    @property
    def is_serial_active(self) -> bool:
        """Serial connection active."""
        return self.serial_connection.is_open

    @property
    def is_success(self) -> bool | StatusCode:
        """Command success."""
        return self._status == StatusCode.SUCCESS

    @property
    def status(self) -> str:
        """Command result."""
        return self._status.name

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

    def _execute(self) -> StatusCode:
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
                status = self._process_data(data)
                checksum = self._read_checksum(address + code + data)
                self._response = address + code + data + checksum
                self._delay()
                break
            except CommandError:
                self._reset_buffers()
            finally:
                tries += 1
        return status if tries < SystemConstants.MAX_RETRIES else StatusCode.MAX_RETRIES_EXCEEDED

    @classmethod
    def unlock(cls):
        """Unlock the command."""
        cls._command_lock = False

    def __repr__(self) -> str:
        """Representation of the command."""
        return f"{self.__class__.__name__}({self.address}, {self.params}, {self.setting})"

    def __str__(self) -> str:
        """String representation of the command."""
        return f"{self.__class__.__name__}({self.address}, {self.params}, {self.setting})"


class WithNoParams(Command):
    """Command with no parameters."""

    def _process_params(self, params: None) -> None:
        return None


class WithEnumParams(Command):
    """Command with parameters."""

    ParamsType = TypeVar("ParamsType", bound=ExtendedIntEnum)

    def _process_params(self, params: ParamsType | None) -> ParamsType:
        return self.ParamsType.default if params is None else params


class WithClassParams(Command):
    """Command with parameters."""

    ParamsType = TypeVar("ParamsType", bound=StepperInput | RangedInt)

    def _process_params(self, params: ParamsType | None) -> ParamsType:
        return self.ParamsType() if params is None else params  # type: ignore


class TakeNoSetting(Command):
    """Command with no setting."""

    def _process_setting(self, setting: None) -> None:
        return None


class TakeSyncSetting(Command):
    """Command with sync setting."""

    def _process_setting(self, setting: SyncFlag | None) -> SyncFlag:
        return SyncFlag.default if setting is None else setting


class TakeStoreSetting(Command):
    """Command with store setting."""

    def _process_setting(self, setting: StoreFlag | None) -> StoreFlag:
        return StoreFlag.default if setting is None else setting


class ReturnSuccess(Command):
    """Command with no data response."""

    _response_length: int = 4

    def _process_data(self, data: bytes) -> StatusCode:
        return StatusCode(int.from_bytes(data, "big"))


class ReturnData(Command):
    """Command with data response."""

    _response_length: int
    ReturnType = TypeVar("ReturnType", bound=StepperOutput)

    def _unpack_data(self, data: bytes) -> ReturnType:
        """Unpack data from response."""
        logger.debug(f"Return type: {self.ReturnType}")
        return self.ReturnType.from_bytes(data)

    def _process_data(self, data: bytes) -> StatusCode:
        self._raw_data = self._unpack_data(data)
        logger.debug(f"Raw data: {self._raw_data}")
        self._data = self._raw_data.data_dict
        return StatusCode.SUCCESS
