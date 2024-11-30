"""Constants for stepper motor protocol"""

from enum import IntEnum
from typing import TypeAlias, NamedTuple
from dataclasses import dataclass, field
from abc import ABC
from pathlib import Path
from logging import getLogger

logger = getLogger(__name__)
COMPort: TypeAlias = str | Path
Data: TypeAlias = bytes | None


class Code(IntEnum):
    """Command codes for stepper motor protocol"""

    # Control Commands
    ENABLE = 0xF3
    JOG = 0xF6
    MOVE = 0xFD
    ESTOP = 0xFE
    SYNC_MOVE = 0xFF

    # Homing Commands
    SET_HOME = 0x93
    HOME = 0x9A
    STOP_HOME = 0x9C
    GET_HOME_STATUS = 0x3B

    # Action Commands
    CAL_ENCODER = 0x06
    CLEAR_POS = 0x0A
    CLEAR_STALL = 0x0E
    RESET = 0x0F

    # Read Commands
    GET_VERSION = 0x1F
    GET_MOTOR_PARAM = 0x20
    GET_PID = 0x21
    GET_VOLTAGE = 0x24
    GET_CURRENT = 0x27
    GET_ENCODER = 0x31
    GET_PULSE_COUNT = 0x32
    GET_TARGET = 0x33
    GET_OPEN_LOOP_SETPOINT = 0x34
    GET_SPEED = 0x35
    GET_POS = 0x36
    GET_ERROR = 0x37
    GET_STATUS = 0x3A

    # Configuration Commands
    SET_MICROSTEP = 0x84
    SET_ID = 0xAE
    SET_MODE = 0x46
    SET_CURRENT = 0x44
    SET_PID = 0x4A
    SAVE_SPEED = 0xF7
    SET_REDUCTION = 0x4F

    # Batch Commands
    GET_CONFIG = 0x42
    SET_CONFIG = 0x48
    GET_HOME_PARAM = 0x22
    SET_HOME_PARAM = 0x4C
    GET_SYS_STATUS = 0x43

    # Error Codes
    ERROR = 0x00


# Protocol constants
class Protocol(IntEnum):
    """Protocol constants for command codes and responses"""

    # Action Commands
    CAL_ENCODER = 0x06
    CLEAR_POS = 0x0A
    CLEAR_STALL = 0x0E
    RESET = 0x0F

    # Read Commands
    GET_CONFIG = 0x42
    GET_CONFIG_RESPONSE = 0x21
    GET_CONFIG_LENGTH = 0x15
    GET_SYS_STATUS = 0x43
    GET_SYS_STATUS_RESPONSE = 0x1F
    GET_SYS_STATUS_LENGTH = 0x09

    # Configuration Commands
    SET_MICROSTEP = 0x84
    SET_ID = 0xAE
    SET_MODE = 0x46
    SET_CURRENT = 0x44
    SET_CONFIG = 0x48
    SET_PID = 0x4A
    SAVE_SPEED = 0xF7
    SET_REDUCTION = 0x4F

    # Control Commands
    ENABLE = 0xFD
    ESTOP = 0xFE
    SYNC_MOVE = 0xFF
    SET_HOME = 0x93
    SET_HOME_PARAM = 0x4C
    STOP_HOME = 0x9C

    # Response codes
    FIXED_CHECKSUM_BYTE = 0x6B
    SUCCESS = 0x02
    CONDITION_ERROR = 0xE2
    ERROR = 0xEE


class MetaParam(NamedTuple):
    """Metadata for a variable class configurations"""

    minimum: int
    maximum: int
    default: int
    digits: int


class RangedInt(int, ABC):
    """A configuration integer constrained to a specific range"""

    meta: MetaParam

    def __new__(cls, value: int | None = None) -> "RangedInt":
        if value is None:
            value = cls.meta.default
        if not cls.meta.minimum <= value <= cls.meta.maximum:
            raise ValueError(
                f"Value must be between {cls.meta.minimum} and {cls.meta.maximum}"
            )
        return super().__new__(cls, value)

    @property
    def bytes(self) -> bytes:
        return self.to_bytes(self.meta.digits, "big")


# Common variables
class Address(RangedInt):
    meta = MetaParam(0, 256, 0, 1)  # 0-255, 1 is default address, 0 is broadcast


class SyncFlag(IntEnum):
    NO_SYNC = 0x00
    SYNC = 0x01

    @property
    def default(self) -> int:
        return self.NO_SYNC


class StoreFlag(IntEnum):
    TEMPORARY = 0x00
    PERMANENT = 0x01

    @property
    def default(self) -> int:
        return self.TEMPORARY


# Movement variables
class EnableFlag(IntEnum):
    DISABLE = 0x00
    ENABLE = 0x01


class Direction(IntEnum):
    CW = 0x00
    CCW = 0x01

    @property
    def default(self) -> int:
        return self.CW


class Speed(RangedInt):
    meta = MetaParam(0, 3000, 0, 2)  # Unit RPM


class Acceleration(RangedInt):
    meta = MetaParam(0, 255, 0, 1)
    # t2 - t1 = (256 - acc) * 50(us)，Vt2 = Vt1 + 1(RPM)


# Position variables
class PulseCount(RangedInt):
    meta = MetaParam(0, 256**4, 0, 4)


class AbsoluteFlag(IntEnum):
    RELATIVE = 0x00
    ABSOLUTE = 0x01

    @property
    def default(self) -> int:
        return self.RELATIVE


# Homing variables
class HomingMode(IntEnum):
    SINGLE_TURN_NEAREST = 0x00
    SINGLE_TURN_DIRECTIONAL = 0x01
    MULTI_TURN_UNLIMITED = 0x02
    MULTI_TURN_LIMITED = 0x03

    @property
    def default(self) -> int:
        return self.SINGLE_TURN_NEAREST


class HomingDirection(IntEnum):
    CW = 0x00
    CCW = 0x01

    @property
    def default(self) -> int:
        return self.CW


class HomingSpeed(RangedInt):
    meta = MetaParam(0, 300, 30, 2)  # Unit: RPM


class HomingTimeout(RangedInt):
    meta = MetaParam(0, 100000, 10000, 4)  # Unit: ms


class CollisionDetectionSpeed(RangedInt):
    meta = MetaParam(0, 500, 300, 2)  # Unit: RPM


class CollisionDetectionCurrent(RangedInt):
    meta = MetaParam(0, 3000, 800, 2)  # Unit: mA


class CollisionDetectionTime(RangedInt):
    meta = MetaParam(0, 500, 60, 2)  # Unit: ms


class AutoHoming(IntEnum):
    DISABLE = 0x00
    ENABLE = 0x01

    @property
    def default(self) -> int:
        return self.DISABLE


@dataclass
class HomingStatus:
    status_code: int
    encoder_ready: bool = field(init=False)
    encoder_calibrated: bool = field(init=False)
    is_homing: bool = field(init=False)
    homing_failed: bool = field(init=False)

    def __post_init__(self) -> None:
        self.encoder_ready = bool(self.status_code & 0x01)
        self.encoder_calibrated = bool(self.status_code & 0x02)
        self.is_homing = bool(self.status_code & 0x04)
        self.homing_failed = bool(self.status_code & 0x08)

    @property
    def __dict__(self) -> dict[str, bool]:
        return {
            "encoder_ready": self.encoder_ready,
            "encoder_calibrated": self.encoder_calibrated,
            "is_homing": self.is_homing,
            "homing_failed": self.homing_failed,
        }


class Kpid(RangedInt):
    meta = MetaParam(0, 256**4, 0, 4)


@dataclass
class StepperStatus:
    ready_status: int
    enabled: bool = field(init=False)
    in_position: bool = field(init=False)
    stalled: bool = field(init=False)
    stall_protection: bool = field(init=False)

    def __post_init__(self) -> None:
        self.enabled = bool(self.ready_status & 0x01)
        self.in_position = bool(self.ready_status & 0x02)
        self.stalled = bool(self.ready_status & 0x04)
        self.stall_protection = bool(self.ready_status & 0x08)

    @property
    def __dict__(self) -> dict[str, bool]:
        return {
            "enabled": self.enabled,
            "in_position": self.in_position,
            "stalled": self.stalled,
            "stall_protection": self.stall_protection,
        }

class MotorType(IntEnum):
    D18 = 0x19
    D09 = 0x32

    @property
    def default(self) -> int:
        return self.D18


class ControlMode(IntEnum):
    PUL_OFF = 0x00
    PUL_OPEN = 0x01
    PUL_FOC = 0x02
    ESI_RCO = 0x03

    @property
    def default(self) -> int:
        return self.PUL_FOC


class CommunicationMode(IntEnum):
    RXTX_OFF = 0x00
    ESI_AL0 = 0x01
    UART = 0x02
    CAN = 0x03

    @property
    def default(self) -> int:
        return self.UART


class EnableLevel(IntEnum):
    LOW = 0x00
    HIGH = 0x01
    HOLD = 0x02

    @property
    def default(self) -> int:
        return self.HOLD


class Microstep(RangedInt):
    meta = MetaParam(0, 256, 16, 1)  # 0x00 is 256 microsteps


class MicrostepInterp(IntEnum):
    DISABLE = 0x00
    ENABLE = 0x01

    @property
    def default(self) -> int:
        return self.ENABLE


class ScreenOff(IntEnum):
    DISABLE = 0x00
    ENABLE = 0x01

    @property
    def default(self) -> int:
        return self.DISABLE


class OpenLoopCurrent(RangedInt):
    meta = MetaParam(0, 2000, 800, 2)  # Unit: mA


class ClosedLoopCurrent(RangedInt):
    meta = MetaParam(0, 4000, 2000, 2)  # Unit: mA


class MaxVoltage(RangedInt):
    meta = MetaParam(0, 5000, 4000, 2)  # Unit: mV


class BaudRate(IntEnum):
    """Supported baud rates for serial communication"""

    BAUD_9600 = 0x00
    BAUD_19200 = 0x01
    BAUD_25000 = 0x02
    BAUD_38400 = 0x03
    BAUD_57600 = 0x04
    BAUD_115200 = 0x05
    BAUD_256000 = 0x06
    BAUD_512000 = 0x07
    BAUD_921600 = 0x08

    @property
    def default(self) -> int:
        return self.BAUD_115200


class CanRate(IntEnum):
    """Supported CAN bus rates in bits per second"""

    CAN_10K = 0x00
    CAN_20K = 0x01
    CAN_50K = 0x02
    CAN_83K = 0x03
    CAN_100K = 0x04
    CAN_125K = 0x05
    CAN_250K = 0x06
    CAN_500K = 0x07
    CAN_800K = 0x08
    CAN_1000K = 0x09

    @property
    def default(self) -> int:
        return self.CAN_500K


class ChecksumMode(IntEnum):
    FIXED = 0x00  # Fixed 0x6B
    XOR = 0x01  # XOR of all bytes
    CRC8 = 0x02  # CRC-8 algorithm

    @property
    def default(self) -> int:
        return self.FIXED


class ResponseMode(IntEnum):
    NONE = 0x00
    RECEIVE = 0x01  # Return ADDR + FD + 9F + CHECKSUM after receiving command
    REACHED = 0x02  # Return ADDR + FD + 9F + CHECKSUM after reaching target
    BOTH = 0x03  # Return ADDR + FD + 9F + CHECKSUM after both
    OTHER = 0x04  # Return other values

    @property
    def default(self) -> int:
        return self.RECEIVE


class StallProtect(IntEnum):
    DISABLE = 0x00
    ENABLE = 0x01

    @property
    def default(self) -> int:
        return self.ENABLE


class StallSpeed(RangedInt):
    meta = MetaParam(0, 500, 28, 2)  # Unit: RPM


class StallCurrent(RangedInt):
    meta = MetaParam(0, 3000, 2400, 2)  # Unit: mA


class StallTime(RangedInt):
    meta = MetaParam(0, 5000, 4000, 2)  # Unit: ms


class PosWindow(RangedInt):
    meta = MetaParam(0, 100, 1, 2)  # Unit: 0.1XDeg


class EnablePin(IntEnum):
    DISABLE = 0x00
    ENABLE = 0x01


class DefaultDir(IntEnum):
    CW = 0x00
    CCW = 0x01


class SpeedReduction(IntEnum):
    DISABLE = 0x00
    ENABLE = 0x01
