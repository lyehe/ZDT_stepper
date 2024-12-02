"""Constants for stepper motor protocol"""

from enum import IntEnum, Enum
from typing import TypeAlias, NamedTuple
from dataclasses import dataclass, field
from abc import ABC, abstractmethod
from pathlib import Path
from logging import getLogger
import math

logger = getLogger(__name__)
COMPort: TypeAlias = str | Path
Data: TypeAlias = bytes | None


class Code(IntEnum):
    """Command codes for stepper motor protocol"""

    # Move Commands
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
    GET_HOME_PARAM = 0x22
    SET_HOME_PARAM = 0x4C

    # Action Commands
    CAL_ENCODER = 0x06
    ZERO_ALL_POSITIONS = 0x0A
    CLEAR_STALL = 0x0E
    FACTORY_RESET = 0x0F

    # Read Commands
    GET_VERSION = 0x1F
    GET_MOTOR_R_H = 0x20
    GET_PID = 0x21
    GET_BUS_VOLTAGE = 0x24
    GET_PHASE_CURRENT = 0x27
    GET_ENCODER_VALUE = 0x31
    GET_PULSE_COUNT = 0x32
    GET_TARGET = 0x33
    GET_OPEN_LOOP_SETPOINT = 0x34
    GET_SPEED = 0x35
    GET_POS = 0x36
    GET_ERROR = 0x37
    GET_STATUS = 0x3A
    GET_CONFIG = 0x42
    GET_SYS_STATUS = 0x43

    # Configuration Commands
    SET_MICROSTEP = 0x84  # checked
    SET_ID = 0xAE  # checked
    SET_LOOP_MODE = 0x46  # checked
    SET_OPEN_LOOP_CURRENT = 0x44  # checked
    SET_CONFIG = 0x48  # checked
    SET_PID = 0x4A  # checked
    SET_START_SPEED = 0xF7  # checked
    SET_REDUCTION = 0x4F  # checked

    # Batch Commands

    # Error Codes
    ERROR = 0x00


# Protocol constants
class Protocol(IntEnum):
    """Protocol constants for command codes and responses"""

    # Move Commands
    ENABLE = 0xAB
    ESTOP = 0x98
    SYNC_MOVE = 0x66

    # Homing Commands
    SET_HOME = 0x88
    STOP_HOME = 0x48
    SET_HOME_PARAM = 0xAE

    # Action Commands
    CAL_ENCODER = 0x45
    ZERO_ALL_POSITIONS = 0x6D
    CLEAR_STALL = 0x52
    FACTORY_RESET = 0x5F

    # Read Commands
    GET_CONFIG = 0x42
    GET_CONFIG_RESPONSE = 0x21
    GET_CONFIG_LENGTH = 0x15
    GET_SYS_STATUS = 0x43
    GET_SYS_STATUS_RESPONSE = 0x1F

    # Configuration Commands
    SET_MICROSTEP = 0x8A
    SET_ID = 0x4B
    SET_LOOP_MODE = 0x69
    SET_OPEN_LOOP_CURRENT = 0x33
    SET_CONFIG = 0xD1
    SET_PID = 0xC3
    SET_START_SPEED = 0x1C
    SET_REDUCTION = 0x71


class StatusCode(IntEnum):
    SUCCESS = 0x02
    CONDITIONAL_ERROR = 0xE2
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
        if not cls.meta.minimum <= value < cls.meta.maximum:
            raise ValueError(
                f"Value must be between {cls.meta.minimum} and {cls.meta.maximum}"
            )
        return super().__new__(cls, value)

    @property
    def bytes(self) -> bytes:
        return self.to_bytes(self.meta.digits, "big")

    @property
    def default(self) -> int:
        return self.meta.default

    @property
    def digits(self) -> int:
        return self.meta.digits


class OptionEnum(IntEnum, ABC):
    """An enumeration with a default value"""

    @abstractmethod
    @property
    def default(self) -> int: ...


class Address(RangedInt):
    """Address of the stepper motor

    Used in all commands
    """

    meta = MetaParam(0, 256, 0, 1)  # 0-255, 1 is default address, 0 is broadcast

    @property
    def default(self) -> int:
        return 1

    @property
    def broadcast(self) -> int:
        return 0


class SyncFlag(OptionEnum):
    """Sync flag for move commands

    Used in: enable, jog, move, estop
    """

    NO_SYNC = 0x00
    SYNC = 0x01

    @property
    def default(self) -> int:
        return self.NO_SYNC


class StoreFlag(OptionEnum):
    """Store flag for configuration commands

    Used in: set_current, set_config, set_pid, save_speed, set_reduction
    """

    TEMPORARY = 0x00
    PERMANENT = 0x01

    @property
    def default(self) -> int:
        return self.TEMPORARY


class EnableFlag(OptionEnum):
    """Enable flag for move commands

    Used in: enable
    """

    DISABLE = 0x00
    ENABLE = 0x01


class Direction(OptionEnum):
    """Direction of the stepper motor

    Used in: jog, move, save_speed
    """

    CW = 0x00
    CCW = 0x01

    @property
    def default(self) -> int:
        return self.CW


class Speed(RangedInt):
    """Speed of the stepper motor

    Used in: jog, move, save_speed
    """

    meta = MetaParam(0, 3000, 0, 2)  # Unit RPM

    @property
    def stop(self) -> int:
        return 0


class Acceleration(RangedInt):
    """Acceleration of the stepper motor

    Used in: jog, move, save_speed
    """

    meta = MetaParam(0, 255, 0, 1)
    # t2 - t1 = (256 - acc) * 50(us)，Vt2 = Vt1 + 1(RPM)


class PulseCount(RangedInt):
    """Pulse count for move commands

    Used in: move
    """

    meta = MetaParam(0, 256**4, 0, 4)


class AbsoluteFlag(OptionEnum):
    """Absolute flag for move commands

    Used in: move
    """

    RELATIVE = 0x00
    ABSOLUTE = 0x01

    @property
    def default(self) -> int:
        return self.RELATIVE


# Homing variables
class HomingMode(OptionEnum):
    """Homing mode for homing commands

    Used in: home, set_home_param
    """

    SINGLE_TURN_NEAREST = 0x00
    SINGLE_TURN_DIRECTIONAL = 0x01
    MULTI_TURN_UNLIMITED = 0x02
    MULTI_TURN_LIMITED = 0x03

    @property
    def default(self) -> int:
        return self.SINGLE_TURN_NEAREST


class HomingDirection(OptionEnum):
    """Homing direction for homing commands

    Used in: set_home_param
    """

    CW = 0x00
    CCW = 0x01

    @property
    def default(self) -> int:
        return self.CW


class HomingSpeed(RangedInt):
    """Homing speed in RPM for homing commands

    Used in: set_home_param
    """

    meta = MetaParam(0, 300, 30, 2)  # Unit: RPM


class HomingTimeout(RangedInt):
    """Homing timeout in milliseconds for homing commands

    Used in: set_home_param
    """

    meta = MetaParam(0, 100000, 10000, 4)  # Unit: ms


class CollisionDetectionSpeed(RangedInt):
    """Collision detection speed in RPM for homing commands

    Used in: set_home_param
    """

    meta = MetaParam(0, 500, 300, 2)  # Unit: RPM


class CollisionDetectionCurrent(RangedInt):
    """Collision detection current in mA for homing commands

    Used in: set_home_param
    """

    meta = MetaParam(0, 3000, 800, 2)  # Unit: mA


class CollisionDetectionTime(RangedInt):
    """Collision detection time in milliseconds for homing commands

    Used in: set_home_param
    """

    meta = MetaParam(0, 500, 60, 2)  # Unit: ms


class AutoHoming(OptionEnum):
    """Auto homing flag for homing commands

    Used in: set_home_param
    """

    DISABLE = 0x00
    ENABLE = 0x01

    @property
    def default(self) -> int:
        return self.DISABLE


@dataclass
class HomingStatus:
    """Homing status for homing commands"""

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
            "homing_active": self.is_homing,
            "homing_failed": self.homing_failed,
        }


class Kpid(RangedInt):
    """Kp, Ki, Kd for PID control

    Used in: set_pid
    """

    meta = MetaParam(0, 256**4, 0, 4)

    @property
    def default_kp(self) -> int:
        return 62000

    @property
    def default_ki(self) -> int:
        return 100

    @property
    def default_kd(self) -> int:
        return 62000


@dataclass
class StepperStatus:
    """Stepper status for read commands"""

    ready_status: int
    enabled: bool = field(init=False)
    in_position: bool = field(init=False)
    stalled: bool = field(init=False)
    stall_protection_active: bool = field(init=False)

    def __post_init__(self) -> None:
        self.enabled = bool(self.ready_status & 0x01)
        self.in_position = bool(self.ready_status & 0x02)
        self.stalled = bool(self.ready_status & 0x04)
        self.stall_protection_active = bool(self.ready_status & 0x08)

    @property
    def __dict__(self) -> dict[str, bool]:
        return {
            "enabled": self.enabled,
            "in_position": self.in_position,
            "stalled": self.stalled,
            "stall_protection_active": self.stall_protection_active,
        }


class MotorType(OptionEnum):
    """Motor type for configuration commands

    Used in: set_config
    """

    D18 = 0x19
    D09 = 0x32

    @property
    def default(self) -> int:
        return self.D18

    @property
    def degrees_per_step(self) -> float:
        return 1.8 if self == MotorType.D18 else 0.9


class ControlMode(OptionEnum):
    """Control mode for configuration commands

    Used in: set_config, set_mode
    """

    PUL_OFF = 0x00
    PUL_OPEN = 0x01
    PUL_FOC = 0x02
    ESI_RCO = 0x03

    @property
    def default(self) -> int:
        return self.PUL_FOC


class CommunicationMode(OptionEnum):
    """Communication mode for configuration commands

    Used in: set_config
    """

    RXTX_OFF = 0x00
    ESI_AL0 = 0x01
    UART = 0x02
    CAN = 0x03

    @property
    def default(self) -> int:
        return self.UART


class EnableLevel(OptionEnum):
    """Enable level for configuration commands

    Used in: set_config
    """

    LOW = 0x00
    HIGH = 0x01
    HOLD = 0x02

    @property
    def default(self) -> int:
        return self.HOLD


class Microstep(RangedInt):
    """Microstep resolution for configuration commands

    Used in: set_config, set_microstep
    """

    meta = MetaParam(0, 256, 16, 1)  # 0x00 is 256 microsteps


class MicrostepInterp(OptionEnum):
    """Microstep interpolation for configuration commands

    Used in: set_config
    """

    DISABLE = 0x00
    ENABLE = 0x01

    @property
    def default(self) -> int:
        return self.ENABLE


class ScreenOff(OptionEnum):
    """Screen off flag for read commands

    Used in: set_config
    """

    DISABLE = 0x00
    ENABLE = 0x01

    @property
    def default(self) -> int:
        return self.DISABLE


class OpenLoopCurrent(RangedInt):
    """Open loop current in mA for configuration commands

    Used in: set_config, set_current
    """

    meta = MetaParam(0, 2000, 800, 2)  # Unit: mA


class ClosedLoopCurrent(RangedInt):
    """Closed loop current in mA for configuration commands

    Used in: set_config
    """

    meta = MetaParam(0, 4000, 2000, 2)  # Unit: mA


class MaxVoltage(RangedInt):
    """Maximum voltage in mV for configuration commands

    Used in: set_config
    """

    meta = MetaParam(0, 5000, 4000, 2)  # Unit: mV


class BaudRate(OptionEnum):
    """Baud rate for serial communication configuration

    Used in: set_config
    """

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


class CanRate(OptionEnum):
    """CAN bus rate in bits per second for configuration

    Used in: set_config
    """

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


class ChecksumMode(OptionEnum):
    """Checksum mode for configuration commands

    Used in: set_config
    """

    FIXED = 0x00  # Fixed 0x6B
    XOR = 0x01  # XOR of all bytes
    CRC8 = 0x02  # CRC-8 algorithm

    @property
    def default(self) -> int:
        return self.FIXED


class LoopMode(OptionEnum):
    """Loop mode for configuration commands

    Used in: set_config, set_mode
    """

    OPEN = 0x01
    CLOSED = 0x02

    @property
    def default(self) -> int:
        return self.CLOSED


class ResponseMode(OptionEnum):
    """Response mode for configuration commands

    Used in: set_config
    """

    NONE = 0x00
    RECEIVE = 0x01  # Return ADDR + FD + 9F + CHECKSUM after receiving command
    REACHED = 0x02  # Return ADDR + FD + 9F + CHECKSUM after reaching target
    BOTH = 0x03  # Return ADDR + FD + 9F + CHECKSUM after both
    OTHER = 0x04  # Return other values

    @property
    def default(self) -> int:
        return self.RECEIVE


class StallProtect(OptionEnum):
    """Stall protection flag for configuration commands

    Used in: set_config
    """

    DISABLE = 0x00
    ENABLE = 0x01

    @property
    def default(self) -> int:
        return self.ENABLE


class StallSpeed(RangedInt):
    """Stall speed in RPM for configuration commands

    Used in: set_config
    """

    meta = MetaParam(0, 500, 28, 2)  # Unit: RPM


class StallCurrent(RangedInt):
    """Stall current in mA for configuration commands

    Used in: set_config
    """

    meta = MetaParam(0, 3000, 2400, 2)  # Unit: mA


class StallTime(RangedInt):
    """Stall time in milliseconds for configuration commands

    Used in: set_config
    """

    meta = MetaParam(0, 5000, 4000, 2)  # Unit: ms


class OnTargetWindow(RangedInt):
    """Position window in 0.1 degrees for on-target detection

    Used in: set_config
    """

    meta = MetaParam(0, 100, 1, 2)  # Unit: 0.1XDeg


class EnablePin(OptionEnum):
    """Enable pin configuration for settings

    Used in: set_config
    """

    DISABLE = 0x00
    ENABLE = 0x01

    @property
    def default(self) -> int:
        return self.ENABLE


class DefaultDir(OptionEnum):
    """Default direction configuration for settings

    Used in: set_config
    """

    CW = 0x00
    CCW = 0x01

    @property
    def default(self) -> int:
        return self.CW


class SpeedReduction(OptionEnum):
    """Speed reduction configuration for settings

    Used in: set_reduction
    """

    DISABLE = 0x00
    ENABLE = 0x01

    @property
    def default(self) -> int:
        return self.DISABLE


class CurrentUnit(OptionEnum):
    """Current unit in settings and output"""

    mA = 1
    A = 1000

    @property
    def default(self) -> int:
        return self.mA


class VoltageUnit(OptionEnum):
    """Voltage unit in settings and output"""

    mV = 1
    V = 1000

    @property
    def default(self) -> int:
        return self.mV


class InductanceUnit(OptionEnum):
    """Inductance unit in output"""

    uH = 1
    mH = 1000
    H = 1000000

    @property
    def default(self) -> int:
        return self.uH


class ResistanceUnit(OptionEnum):
    """Resistance unit in output"""

    mOhm = 1
    Ohm = 1000

    @property
    def default(self) -> int:
        return self.mOhm


class AngleUnit(Enum):
    """Angle unit in output"""

    deg = 360
    rad = 2 * math.pi

    @property
    def default(self) -> int:
        return self.deg


class TimeUnit(OptionEnum):
    """Time unit in output"""

    ms = 1
    s = 1000

    @property
    def default(self) -> int:
        return self.ms

@dataclass
class DefaultParameters:
    """Default parameters for configuration commands"""

    open_loop_current: int = 1000
    max_closed_loop_current: int = 1000
    max_voltage: int = 4000
    stall_speed: int = 28
    stall_current: int = 2400
    stall_time: int = 4000
    pos_window: int = 1

