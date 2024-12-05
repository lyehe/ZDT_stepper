"""Constants for stepper motor protocol."""

import math
from dataclasses import dataclass, field
from enum import Enum, IntEnum
from logging import getLogger
from pathlib import Path
from typing import NamedTuple, TypeAlias

logger = getLogger(__name__)
COMPort: TypeAlias = str | Path
Data: TypeAlias = bytes | None


class ExtendedIntEnum(IntEnum):
    """An integer enumeration with bytes representation."""

    @property
    def bytes(self) -> bytes:
        """Bytes representation."""
        return self.value.to_bytes(1, "big")

@dataclass(frozen=True)
class SystemConstants:
    """System constants for stepper motor protocol.
    
    :param SERIAL_TIMEOUT: Timeout for serial communication in seconds
    :param MAX_RETRIES: Maximum number of retries for failed commands
    """
    SERIAL_TIMEOUT: float = field(default=0.005)
    MAX_RETRIES: int = field(default=3)


class Code(ExtendedIntEnum):
    """Command codes for stepper motor protocol."""

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

    ERROR = 0x00  # Error Codes


# Protocol constants
class Protocol(ExtendedIntEnum):
    """Protocol constants for command codes and responses."""

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


class StatusCode(ExtendedIntEnum):
    """Status codes for read commands."""

    FIXED_CHECKSUM_BYTE = 0x6B
    SUCCESS = 0x02
    CONDITIONAL_ERROR = 0xE2
    ERROR = 0xEE


class MetaParam(NamedTuple):
    """params for a variable class configurations."""

    minimum: int
    maximum: int
    default: int
    digits: int


class RangedInt(int):
    """A configuration integer constrained to a specific range."""

    meta: MetaParam

    def __new__(cls, value: int | None = None) -> "RangedInt":
        """Create a new instance of RangedInt that is within the range."""
        if value is None:
            value = cls.meta.default
        if not cls.meta.minimum <= value <= cls.meta.maximum:
            raise ValueError(f"Value must be between {cls.meta.minimum} and {cls.meta.maximum}")
        return super().__new__(cls, value)

    @property
    def bytes(self) -> bytes:
        """Bytes representation."""
        return self.to_bytes(self.meta.digits, "big")

    @property
    def default(self) -> int:
        """Default value."""
        return self.meta.default

    @property
    def digits(self) -> int:
        """Number of digits."""
        return self.meta.digits


class Address(RangedInt):
    """Address of the stepper motor.

    Used in all commands.
    """

    meta = MetaParam(minimum=0, maximum=255, default=1, digits=1)

    # 0-255, 1 is default address, 0 is broadcast

    @property
    def default(self) -> int:
        """Default address."""
        return 1

    @property
    def broadcast(self) -> int:
        """Broadcast address."""
        return 0


class SyncFlag(ExtendedIntEnum):
    """Sync flag for move commands.

    Used in: enable, jog, move, estop
    """

    NO_SYNC = 0x00
    SYNC = 0x01
    default = NO_SYNC


class StoreFlag(ExtendedIntEnum):
    """Store flag for configuration commands.

    Used in: set_current, set_config, set_pid, save_speed, set_reduction
    """

    TEMPORARY = 0x00
    PERMANENT = 0x01
    default = TEMPORARY


class EnableFlag(ExtendedIntEnum):
    """Enable flag for move commands.

    Used in: enable
    """

    DISABLE = 0x00
    ENABLE = 0x01


class Direction(ExtendedIntEnum):
    """Direction of the stepper motor.

    Used in: jog, move, save_speed
    """

    CW = 0x00
    CCW = 0x01
    default = CW


class Speed(RangedInt):
    """Speed of the stepper motor.

    Used in: jog, move, save_speed
    """

    meta = MetaParam(minimum=0, maximum=3000, default=0, digits=2)  # Unit RPM

    @property
    def stop(self) -> int:
        """Stop speed."""
        return 0


class Acceleration(RangedInt):
    """Acceleration of the stepper motor.

    Used in: jog, move, save_speed
    """

    meta = MetaParam(minimum=0, maximum=255, default=0, digits=1)
    # t2 - t1 = (256 - acc) * 50(us)，Vt2 = Vt1 + 1(RPM)


class PulseCount(RangedInt):
    """Pulse count for move commands.

    Used in: move
    """

    meta = MetaParam(minimum=0, maximum=256**4 - 1, default=0, digits=4)


class AbsoluteFlag(ExtendedIntEnum):
    """Absolute flag for move commands.

    Used in: move
    """

    RELATIVE = 0x00
    ABSOLUTE = 0x01
    default = RELATIVE


# Homing variables
class HomingMode(ExtendedIntEnum):
    """Homing mode for homing commands.

    Used in: home, set_home_param
    """

    SINGLE_TURN_NEAREST = 0x00
    SINGLE_TURN_DIRECTIONAL = 0x01
    MULTI_TURN_UNLIMITED = 0x02
    MULTI_TURN_LIMITED = 0x03
    default = SINGLE_TURN_NEAREST


class HomingDirection(ExtendedIntEnum):
    """Homing direction for homing commands.

    Used in: set_home_param
    """

    CW = 0x00
    CCW = 0x01
    default = CW


class HomingSpeed(RangedInt):
    """Homing speed in RPM for homing commands.

    Used in: set_home_param
    """

    meta = MetaParam(minimum=0, maximum=300, default=30, digits=2)  # Unit: RPM


class HomingTimeout(RangedInt):
    """Homing timeout in milliseconds for homing commands.

    Used in: set_home_param
    """

    meta = MetaParam(minimum=0, maximum=100000, default=10000, digits=4)  # Unit: ms


class CollisionDetectionSpeed(RangedInt):
    """Collision detection speed in RPM for homing commands.

    Used in: set_home_param
    """

    meta = MetaParam(minimum=0, maximum=500, default=300, digits=2)  # Unit: RPM


class CollisionDetectionCurrent(RangedInt):
    """Collision detection current in mA for homing commands.

    Used in: set_home_param
    """

    meta = MetaParam(minimum=0, maximum=3000, default=800, digits=2)  # Unit: mA


class CollisionDetectionTime(RangedInt):
    """Collision detection time in milliseconds for homing commands.

    Used in: set_home_param
    """

    meta = MetaParam(minimum=0, maximum=500, default=60, digits=2)  # Unit: ms


class AutoHoming(ExtendedIntEnum):
    """Auto homing flag for homing commands.

    Used in: set_home_param
    """

    DISABLE = 0x00
    ENABLE = 0x01
    default = DISABLE


@dataclass
class HomingStatus:
    """Homing status for homing commands."""

    status_code: int
    encoder_ready: bool = field(init=False)
    encoder_calibrated: bool = field(init=False)
    is_homing: bool = field(init=False)
    homing_failed: bool = field(init=False)

    def __post_init__(self) -> None:
        """Post initialization to set flags."""
        self.encoder_ready = bool(self.status_code & 0x01)
        self.encoder_calibrated = bool(self.status_code & 0x02)
        self.is_homing = bool(self.status_code & 0x04)
        self.homing_failed = bool(self.status_code & 0x08)

    @property
    def __dict__(self) -> dict[str, bool]:
        """Dictionary representation of the homing status."""
        return {
            "encoder_ready": self.encoder_ready,
            "encoder_calibrated": self.encoder_calibrated,
            "homing_active": self.is_homing,
            "homing_failed": self.homing_failed,
        }


class Kpid(RangedInt):
    """Kp, Ki, Kd for PID control.

    Used in: set_pid
    """

    meta = MetaParam(minimum=0, maximum=256**4 - 1, default=0, digits=4)

    @property
    def default_kp(self) -> int:
        """Default Kp."""
        return 62000

    @property
    def default_ki(self) -> int:
        """Default Ki."""
        return 100

    @property
    def default_kd(self) -> int:
        """Default Kd."""
        return 62000


@dataclass
class StepperStatus:
    """Stepper status for read commands."""

    ready_status: int
    enabled: bool = field(init=False)
    in_position: bool = field(init=False)
    stalled: bool = field(init=False)
    stall_protection_active: bool = field(init=False)

    def __post_init__(self) -> None:
        """Post initialization to set flags."""
        self.enabled = bool(self.ready_status & 0x01)
        self.in_position = bool(self.ready_status & 0x02)
        self.stalled = bool(self.ready_status & 0x04)
        self.stall_protection_active = bool(self.ready_status & 0x08)

    @property
    def __dict__(self) -> dict[str, bool]:
        """Dictionary representation of the stepper status."""
        return {
            "enabled": self.enabled,
            "in_position": self.in_position,
            "stalled": self.stalled,
            "stall_protection_active": self.stall_protection_active,
        }


class MotorType(ExtendedIntEnum):
    """Motor type for configuration commands.

    Used in: set_config
    """

    D18 = 0x19  # 1.8 degrees per step
    D09 = 0x32  # 0.9 degrees per step
    default = D18

    @property
    def degrees_per_step(self) -> float:
        """Degrees per step."""
        return 1.8 if self == MotorType.D18 else 0.9


class ControlMode(ExtendedIntEnum):
    """Control mode for configuration commands.

    Used in: set_config, set_mode
    """

    PUL_OFF = 0x00
    PUL_OPEN = 0x01
    PUL_FOC = 0x02
    ESI_RCO = 0x03
    default = PUL_FOC


class CommunicationMode(ExtendedIntEnum):
    """Communication mode for configuration commands.

    Used in: set_config
    """

    RXTX_OFF = 0x00
    ESI_AL0 = 0x01
    UART = 0x02
    CAN = 0x03
    default = UART


class EnableLevel(ExtendedIntEnum):
    """Enable level for configuration commands.

    Used in: set_config
    """

    LOW = 0x00
    HIGH = 0x01
    HOLD = 0x02
    default = HOLD


class Microstep(RangedInt):
    """Microstep resolution for configuration commands.

    Used in: set_config, set_microstep
    """

    meta = MetaParam(minimum=0, maximum=255, default=16, digits=1)  # 0x00 is 256 microsteps


class MicrostepInterp(ExtendedIntEnum):
    """Microstep interpolation for configuration commands.

    Used in: set_config
    """

    DISABLE = 0x00
    ENABLE = 0x01
    default = ENABLE


class ScreenOff(ExtendedIntEnum):
    """Screen off flag for read commands.

    Used in: set_config
    """

    DISABLE = 0x00
    ENABLE = 0x01
    default = DISABLE


class OpenLoopCurrent(RangedInt):
    """Open loop current in mA for configuration commands.

    Used in: set_config, set_current
    """

    meta = MetaParam(minimum=0, maximum=2000, default=800, digits=2)  # Unit: mA


class ClosedLoopCurrent(RangedInt):
    """Closed loop current in mA for configuration commands.

    Used in: set_config
    """

    meta = MetaParam(minimum=0, maximum=4000, default=2000, digits=2)  # Unit: mA


class MaxVoltage(RangedInt):
    """Maximum voltage in mV for configuration commands.

    Used in: set_config
    """

    meta = MetaParam(minimum=0, maximum=5000, default=4000, digits=2)  # Unit: mV


class BaudRate(ExtendedIntEnum):
    """Baud rate for serial communication configuration.

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
    default = BAUD_115200


class CanRate(ExtendedIntEnum):
    """CAN bus rate in bits per second for configuration.

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
    default = CAN_500K


class ChecksumMode(ExtendedIntEnum):
    """Checksum mode for configuration commands.

    Used in: set_config
    """

    FIXED = 0x00  # Fixed 0x6B
    XOR = 0x01  # XOR of all bytes
    CRC8 = 0x02  # CRC-8 algorithm

    default = FIXED  # Class variable for default value


class LoopMode(ExtendedIntEnum):
    """Loop mode for configuration commands.

    Used in: set_config, set_mode
    """

    OPEN = 0x01
    CLOSED = 0x02
    default = CLOSED


class ResponseMode(ExtendedIntEnum):
    """Response mode for configuration commands.

    Used in: set_config
    """

    NONE = 0x00
    RECEIVE = 0x01  # Return ADDR + FD + 9F + CHECKSUM after receiving command
    REACHED = 0x02  # Return ADDR + FD + 9F + CHECKSUM after reaching target
    BOTH = 0x03  # Return ADDR + FD + 9F + CHECKSUM after both
    OTHER = 0x04  # Return other values
    default = RECEIVE


class StallProtect(ExtendedIntEnum):
    """Stall protection flag for configuration commands.

    Used in: set_config
    """

    DISABLE = 0x00
    ENABLE = 0x01
    default = ENABLE


class StallSpeed(RangedInt):
    """Stall speed in RPM for configuration commands.

    Used in: set_config
    """

    meta = MetaParam(minimum=0, maximum=500, default=28, digits=2)  # Unit: RPM


class StallCurrent(RangedInt):
    """Stall current in mA for configuration commands.

    Used in: set_config
    """

    meta = MetaParam(minimum=0, maximum=3000, default=2400, digits=2)  # Unit: mA


class StallTime(RangedInt):
    """Stall time in milliseconds for configuration commands.

    Used in: set_config
    """

    meta = MetaParam(minimum=0, maximum=5000, default=4000, digits=2)  # Unit: ms


class OnTargetWindow(RangedInt):
    """Position window in 0.1 degrees for on-target detection.

    Used in: set_config
    """

    meta = MetaParam(minimum=0, maximum=100, default=1, digits=2)  # Unit: 0.1XDeg


class EnablePin(ExtendedIntEnum):
    """Enable pin configuration for settings.

    Used in: set_config
    """

    DISABLE = 0x00
    ENABLE = 0x01
    default = ENABLE


class DefaultDir(ExtendedIntEnum):
    """Default direction configuration for settings.

    Used in: set_config
    """

    CW = 0x00
    CCW = 0x01
    default = CW


class SpeedReduction(ExtendedIntEnum):
    """Speed reduction configuration for settings.

    Used in: set_reduction
    """

    DISABLE = 0x00
    ENABLE = 0x01
    default = DISABLE


class CurrentUnit(ExtendedIntEnum):
    """Current unit in settings and output.

    Used in: set_config
    """

    mA = 1  # noqa: N815
    A = 1000
    default = mA


class VoltageUnit(ExtendedIntEnum):
    """Voltage unit in settings and output.

    Used in: set_config
    """

    mV = 1  # noqa: N815
    V = 1000
    default = mV


class InductanceUnit(ExtendedIntEnum):
    """Inductance unit in output.

    Used in: set_config
    """

    uH = 1  # noqa: N815
    mH = 1000  # noqa: N815
    H = 1000000
    default = uH


class ResistanceUnit(ExtendedIntEnum):
    """Resistance unit in output.

    Used in: set_config
    """

    mOhm = 1  # noqa: N815
    Ohm = 1000
    default = mOhm


class AngleUnit(Enum):
    """Angle unit in output.

    Used in: set_config
    """

    deg = 360
    rad = 2 * math.pi
    default = deg


class TimeUnit(ExtendedIntEnum):
    """Time unit in output.

    Used in: set_config
    """

    ms = 1  # noqa: N815
    s = 1000
    default = ms


@dataclass
class DefaultParameters:
    """Default parameters for configuration commands."""

    open_loop_current: int = 1000
    max_closed_loop_current: int = 1000
    max_voltage: int = 4000
    stall_speed: int = 28
    stall_current: int = 2400
    stall_time: int = 4000
    position_window: int = 1
