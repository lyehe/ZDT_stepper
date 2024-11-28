from enum import IntEnum, Enum, auto
from typing import Final, TypeAlias, Literal, namedtuple
from pathlib import Path

COMPort: TypeAlias = str | Path
BaudRate: TypeAlias = Literal[9600, 19200, 38400, 57600, 115200]
CanRate: TypeAlias = Literal[
    10000, 20000, 50000, 100000, 125000, 250000, 500000, 1000000
]

Data: TypeAlias = bytes | None
Address: TypeAlias = int
Speed: TypeAlias = int
Acceleration: TypeAlias = int
PulseCount: TypeAlias = int
CommandValue = namedtuple(
    "Command",
    ["code", "data_length", "data_format", "response_length", "response_format"],
)


# Command codes
class CMD_CODE(Enum):
    """Command codes for stepper motor protocol"""

    # Control Commands
    ENABLE = CommandValue(0xF3, 3, (1, 1, 1), 1, (1))
    SPEED = CommandValue(0xF6, 5, (1, 1, 1, 1, 1), 1, (1))
    POSITION = CommandValue(0xFD, 10, (1, 2, 1, 4, 1, 1), 1, (1))
    EMERGENCY_STOP = CommandValue(0xFE, 2, (1, 1), 1, (1))
    SYNC_MOVE = CommandValue(0xFF, 1, (1), 1, (1))

    # Homing Commands
    SET_HOME = CommandValue(0x93, 2, (1, 1), 1, (1))
    HOMING = CommandValue(0x9A, 2, (1, 1), 1, (1))
    ABORT_HOME = CommandValue(0x9C, 1, (1), 1, (1))
    READ_HOMING_STATUS = CommandValue(0x22, 15, (1, 1, 2, 4, 2, 2, 2, 1), (1, 1, 2, 4, 2, 2, 2, 1))

    # Action Commands
    CALIBRATE = 0x06
    CLEAR_POS = 0x0A
    CLEAR_STALL = 0x0E
    FACTORY_RESET = 0x0F

    # Read Commands
    READ_VERSION = 0x1F
    READ_PHASE_PARAMS = 0x20
    READ_PID = 0x21
    READ_BUS_VOLTAGE = 0x24
    READ_PHASE_CURRENT = 0x27
    READ_CALIBRATED_ENCODER = 0x31
    READ_INPUT_PULSE_COUNT = 0x32
    READ_TARGET = 0x33  # Target position
    READ_REALTIME_TARGET = 0x34
    READ_REALTIME_SPEED = 0x35
    READ_POSITION = 0x36
    READ_POSITION_ERROR = 0x37
    READ_STATUS = 0x3A
    READ_HOMING_STATUS = 0x3B
    READ_DRIVE_CONFIG = 0x42
    READ_SYSTEM_STATUS = 0x43

    # Configuration Commands
    MODIFY_SUBDIVISION = 0x84
    MODIFY_ID_ADDRESS = 0xAE
    SWITCH_LOOP_MODE = 0x46
    MODIFY_OPEN_LOOP_CURRENT = 0x44
    MODIFY_PID_PARAMS = 0x4A
    STORE_SPEED_PARAMS = 0xF7
    MODIFY_SPEED_SCALE = 0x4F


# Protocol constants
class PROTOCOL(Enum):
    ENABLE_CODE = bytes([0xAB])
    STOP_CODE = bytes([0x98])
    SYNC_MOVE_CODE = bytes([0x66])
    SET_HOME_CODE = bytes([0x88])
    ABORT_HOME_CODE = bytes([0x48])

    CHECKSUM_BYTE = bytes([0x6B])
    SUCCESS_CODE = bytes([0x02])
    CONDITION_ERROR = bytes([0xE2])
    WRONG_COMMAND_ERROR = bytes([0xEE])


# Configuration enums
class ENABLE_FLAG(Enum):
    DISABLE = bytes([0])
    ENABLE = bytes([1])


class DIRECTION(Enum):
    CW = bytes([0])
    CCW = bytes([1])


class STORE_FLAG(Enum):
    TEMPORARY = bytes([0])
    PERMANENT = bytes([1])


class SYNC_FLAG(Enum):
    NO_SYNC = bytes([0])
    SYNC = bytes([1])


class ABSOLUTE_FLAG(Enum):
    RELATIVE = bytes([0])
    ABSOLUTE = bytes([1])


class HOMING_MODE(Enum):
    SINGLE_TURN_NEAREST = bytes([0])
    SINGLE_TURN_DIRECTIONAL = bytes([1])
    MULTI_TURN_UNLIMITED = bytes([2])
    MULTI_TURN_LIMITED = bytes([3])


class CHECKSUM_TYPE(Enum):
    """Checksum calculation methods"""

    FIXED = auto()  # Fixed 0x6B
    XOR = auto()  # XOR of all bytes
    CRC8 = auto()  # CRC-8 algorithm


DEFAULT_ADDRESS: Final[Address] = 1
DEFAULT_CHECKSUM_TYPE: Final[CHECKSUM_TYPE] = CHECKSUM_TYPE.FIXED
DEFAULT_DIRECTION: Final[DIRECTION] = DIRECTION.CW
DEFAULT_SYNC_FLAG: Final[SYNC_FLAG] = SYNC_FLAG.NO_SYNC
DEFAULT_STORE_FLAG: Final[STORE_FLAG] = STORE_FLAG.TEMPORARY
DEFAULT_ABSOLUTE_FLAG: Final[ABSOLUTE_FLAG] = ABSOLUTE_FLAG.RELATIVE
DEFAULT_HOMING_MODE: Final[HOMING_MODE] = HOMING_MODE.SINGLE_TURN_NEAREST
