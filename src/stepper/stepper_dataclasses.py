from dataclasses import dataclass
from .stepper_constants import (
    CMD_CODE,
    PROTOCOL,
    Address,
    Data,
    CHECKSUM_TYPE,
)
from .stepper_utilities import calculate_checksum
from .stepper_exceptions import CommandError, ValidationError


@dataclass
class MotorStatus:
    """Motor status information"""

    enabled: bool = False
    position_reached: bool = False
    stalled: bool = False
    protection_triggered: bool = False

    @classmethod
    def from_byte(cls, status: int) -> "MotorStatus":
        return cls(
            enabled=bool(status & 0x01),
            position_reached=bool(status & 0x02),
            stalled=bool(status & 0x04),
            protection_triggered=bool(status & 0x08),
        )

    def as_dict(self) -> dict[str, bool]:
        return {
            "enabled": self.enabled,
            "position_reached": self.position_reached,
            "stalled": self.stalled,
            "protection_triggered": self.protection_triggered,
        }


@dataclass
class Position:
    """Motor position information"""

    steps: int
    revolutions: int
    degrees: float

    @classmethod
    def from_bytes(cls, data: bytes) -> "Position":
        if len(data) < 4:
            raise ValueError("Position data must be at least 4 bytes")

        sign = -1 if data[0] else 1
        steps = sign * int.from_bytes(data[1:], "big")
        revolutions = steps // 65536
        degrees = (abs(steps) % 65536) * 360 / 65536
        return cls(steps=steps, revolutions=revolutions, degrees=degrees)

    def __str__(self) -> str:
        return f"Position(steps={self.steps}, revolutions={self.revolutions}, degrees={self.degrees:.1f})"


@dataclass
class EncoderCalibrationStatus:
    """Encoder calibration status information"""

    is_calibrated: bool = False
    calibration_table_ready: bool = False
    zero_position_set: bool = False

    @classmethod
    def from_byte(cls, status: int) -> "EncoderCalibrationStatus":
        return cls(
            is_calibrated=bool(status & 0x01),
            calibration_table_ready=bool(status & 0x02),
            zero_position_set=bool(status & 0x04),
        )

    def as_dict(self) -> dict[str, bool]:
        return {
            "is_calibrated": self.is_calibrated,
            "calibration_table_ready": self.calibration_table_ready,
            "zero_position_set": self.zero_position_set,
        }


@dataclass(frozen=True)
class CommandOutput:
    """Result of a motor command execution"""

    response_bytes: bytes
    expected_device_number: Address
    expected_command_code: CMD_CODE
    expected_response_length: int
    checksum_type: CHECKSUM_TYPE

    @property
    def device_number(self) -> int:
        return self.response_bytes[0]

    @property
    def command_code(self) -> int:
        return self.response_bytes[1]

    @property
    def data(self) -> bytes:
        return self.response_bytes[2:-1]

    @property
    def checksum(self) -> int:
        return self.response_bytes[-1]

    def __post_init__(self):
        if len(self.response_bytes) < 3:  # Need at least: Address + CMD + Checksum
            raise ValidationError(
                f"Response too short: {len(self.response_bytes)} bytes"
            )
        if self.device_number != self.expected_device_number:
            raise ValidationError("Wrong device number in response")
        if self.command_code != self.expected_command_code:
            raise ValidationError("Wrong command in response")
        expected_checksum = calculate_checksum(
            self.checksum_type, self.response_bytes[:-1]
        )
        if expected_checksum != self.checksum:
            raise ValidationError("Invalid checksum in response")

    @property
    def is_wrong_condition(self) -> bool:
        return self.data == PROTOCOL.CONDITION_ERROR

    @property
    def is_wrong_command(self) -> bool:
        return self.data == PROTOCOL.WRONG_COMMAND_ERROR


@dataclass
class CommandInput:
    """Base class for motor commands"""

    device_number: Address
    code: CMD_CODE
    data: Data
    requires_enabled: bool
    description: str
    checksum_type: CHECKSUM_TYPE
    expected_response_length: int

    def __post_init__(self):
        if self.device_number not in range(16):
            raise CommandError(f"Invalid device number: {self.device_number}")
        if self.code not in CMD_CODE:
            raise CommandError(f"Invalid command code: {self.code}")
        if not self.description:
            self.description = CMD_CODE(self.code).name

    @property
    def command(self) -> bytes:
        """Build complete command bytes"""
        cmd = bytes([self.device_number, self.code]) + self.data
        checksum = calculate_checksum(self.checksum_type, cmd)
        return cmd + bytes([checksum])

    def parse_command_output(self, response_bytes: bytes) -> CommandOutput:
        """Parse command response"""
        expected_device_number = self.device_number if self.device_number != 0 else 1
        return CommandOutput(
            response_bytes=response_bytes,
            expected_device_number=expected_device_number,
            expected_command_code=self.code,
            expected_response_length=self.expected_response_length,
            checksum_type=self.checksum_type,
        )
