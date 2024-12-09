"""params container classes."""

import json
from abc import ABC, abstractmethod
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import TypeAlias

import yaml
from serial import Serial

from .configs import (
    AbsoluteFlag,
    Acceleration,
    Address,
    AngleUnit,
    AutoHoming,
    BaudRate,
    CanRate,
    ChecksumMode,
    ClosedLoopCurrent,
    CollisionDetectionCurrent,
    CollisionDetectionSpeed,
    CollisionDetectionTime,
    CommunicationMode,
    ControlMode,
    CurrentUnit,
    Direction,
    EnableLevel,
    EnablePin,
    HomingDirection,
    HomingMode,
    HomingSpeed,
    HomingTimeout,
    InductanceUnit,
    Kpid,
    LoopMode,
    MaxVoltage,
    Microstep,
    MicrostepInterp,
    MotorType,
    OnTargetWindow,
    OpenLoopCurrent,
    PulseCount,
    ResistanceUnit,
    ResponseMode,
    ScreenOff,
    Speed,
    SpeedReduction,
    SpeedUnit,
    StallCurrent,
    StallProtect,
    StallSpeed,
    StallTime,
    StoreFlag,
    SyncFlag,
    TimeUnit,
    VoltageUnit,
)

__all__ = [
    "DeviceParams",
    "JogParams",
    "PositionParams",
    "HomingParams",
    "HomingStatus",
    "VersionParams",
    "MotorRHParams",
    "PIDParams",
    "BusVoltageParams",
    "PhaseCurrentParams",
    "EncoderParams",
    "PulseCountParams",
    "TargetPositionParams",
    "OpenLoopTargetPositionParams",
    "RealTimeSpeedParams",
    "RealTimePositionParams",
    "PositionErrorParams",
    "StepperStatus",
    "StartSpeedParams",
    "ConfigParams",
    "SystemParams",
    "SerialParams",
    "DeviceParams",
    "InputParams",
    "OutputParams",
    "InputOutputParams",
]


PathVar: TypeAlias = Path | str


def to_int(input: bytes) -> int:
    """Convert bytes to int."""
    return int.from_bytes(input, "big")


def to_signed_int(input: bytes) -> int:
    """Convert long bytes to signed int."""
    sign = -1 if input[0] == 1 else 1
    return sign * to_int(input[1:])


@dataclass(frozen=True)
class SerialParams:
    """Serial connection parameters.

    :param port: Serial port name (e.g., 'COM1', '/dev/ttyUSB0')
    :param baudrate: Communication speed in bits per second
    :param timeout: Read timeout in seconds
    :param write_timeout: Write timeout in seconds
    :param parity: Parity checking (PARITY_NONE, PARITY_EVEN, PARITY_ODD)
    :param stopbits: Number of stop bits (STOPBITS_ONE, STOPBITS_TWO)
    :param bytesize: Number of data bits
    """

    port: str
    baudrate: int = 115200
    timeout: float | None = 1.0
    write_timeout: float | None = 1.0
    parity: str = "N"  # PARITY_NONE
    stopbits: int = 1  # STOPBITS_ONE
    bytesize: int = 8  # EIGHTBITS
    _connection: Serial | None = field(default=None, init=False)

    @property
    def connection(self) -> Serial:
        """Get or create a serial connection with the specified parameters."""
        if self._connection is None:
            self._connection = Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                write_timeout=self.write_timeout,
                parity=self.parity,
                stopbits=self.stopbits,
                bytesize=self.bytesize,
            )
        return self._connection


@dataclass
class DeviceParams:
    """Device parameter class.

    :param serial_connection: Serial connection object
    :param address: Device address
    :param checksum_mode: Checksum mode
    :param delay: Communication delay in seconds
    """

    serial_connection: Serial
    address: Address | int = Address.default
    checksum_mode: ChecksumMode = ChecksumMode.default
    delay: float | None = None

    def __post_init__(self):
        """Post initialization to set address."""
        if isinstance(self.address, int):
            self.address = Address(self.address)


@dataclass
class StepperParams:
    """Stepper parameter class."""

    def to_dict(self) -> dict:
        """Convert to dictionary."""
        return asdict(self)

    @classmethod
    def from_dict(cls, data: dict) -> "StepperParams":
        """Convert from dictionary."""
        return cls(**data)

    def to_json(self, path: PathVar) -> None:
        """Convert to JSON."""
        with open(path, "w") as f:
            json.dump(self.to_dict(), f, indent=2)

    @classmethod
    def from_json(cls, path: PathVar) -> "StepperParams":
        """Convert from JSON."""
        with open(path) as f:
            return cls.from_dict(json.load(f))

    def to_yaml(self, path: PathVar) -> None:
        """Convert to YAML."""
        with open(path, "w") as f:
            yaml.dump(self.to_dict(), f)

    @classmethod
    def from_yaml(cls, path: PathVar) -> "StepperParams":
        """Convert from YAML."""
        with open(path) as f:
            return cls.from_dict(yaml.safe_load(f))


@dataclass
class StepperInput(StepperParams, ABC):
    """Stepper params input class."""

    @abstractmethod
    def bytes(self) -> bytes:
        """Bytes representation."""
        pass


@dataclass
class StepperOutput(StepperParams, ABC):
    """Stepper params output class."""

    @classmethod
    @abstractmethod
    def from_bytes(cls, data: bytes) -> "StepperOutput":
        """Convert from bytes."""
        pass


@dataclass
class JogParams(StepperInput):
    """Velocity data params."""

    direction: Direction = Direction.default
    speed: Speed | int = Speed.default
    acceleration: Acceleration | int = Acceleration.default

    def __post_init__(self):
        """Post initialization to set direction and speed."""
        if isinstance(self.speed, int):
            if self.speed < 0:
                self.direction = Direction.CCW
                self.speed = Speed(-self.speed)
            else:
                self.direction = Direction.CW
                self.speed = Speed(self.speed)
        if isinstance(self.acceleration, int):
            self.acceleration = Acceleration(self.acceleration)

    @property
    def bytes(self) -> bytes:
        """Bytes representation."""
        return bytes([self.direction, *self.speed.bytes, self.acceleration])

    @property
    def data_dict(self) -> dict:
        """Dictionary representation."""
        return {
            "direction": self.direction.name,
            "speed": self.speed,
            "acceleration": self.acceleration,
        }


@dataclass
class PositionParams(StepperInput):
    """Position data params."""

    direction: Direction = Direction.default
    speed: Speed | int = Speed.default
    acceleration: Acceleration | int = Acceleration.default
    pulse_count: PulseCount | int = PulseCount.default
    absolute: AbsoluteFlag = AbsoluteFlag.default

    def __post_init__(self):
        """Post initialization to set direction and speed."""
        if isinstance(self.speed, int):
            self.speed = Speed(self.speed)
        if isinstance(self.acceleration, int):
            self.acceleration = Acceleration(self.acceleration)
        if isinstance(self.pulse_count, int):
            if self.pulse_count < 0:
                self.direction = Direction.CCW
                self.pulse_count = PulseCount(-self.pulse_count)
            else:
                self.direction = Direction.CW
                self.pulse_count = PulseCount(self.pulse_count)

    @property
    def bytes(self) -> bytes:
        """Bytes representation."""
        return bytes(
            [
                self.direction,
                *self.speed.bytes,
                self.acceleration,
                *self.pulse_count.bytes,
                self.absolute,
            ]
        )

    @property
    def data_dict(self) -> dict:
        """Dictionary representation."""
        return {
            "direction": self.direction.name,
            "speed": self.speed,
            "acceleration": self.acceleration,
            "pulse_count": self.pulse_count,
            "absolute": self.absolute.name,
        }


@dataclass
class HomingParams(StepperInput, StepperOutput):
    """Home parameters params.

    :param store: Store flag
    :param homing_mode: Homing mode
    :param homing_direction: Homing direction
    :param homing_speed: Homing speed
    :param homing_timeout: Homing timeout
    :param collision_detection_speed: Collision detection speed
    :param collision_detection_current: Collision detection current
    :param collision_detection_time: Collision detection time
    :param auto_home: Auto home flag
    """

    homing_mode: HomingMode
    homing_direction: HomingDirection
    homing_speed: HomingSpeed
    homing_timeout: HomingTimeout
    collision_detection_speed: CollisionDetectionSpeed | int
    collision_detection_current: CollisionDetectionCurrent | int
    collision_detection_time: CollisionDetectionTime | int
    auto_home: AutoHoming

    current_unit: CurrentUnit = CurrentUnit.default
    time_unit: TimeUnit = TimeUnit.default
    speed_unit: SpeedUnit = SpeedUnit.default

    def __post_init__(self):
        """Post initialization to set collision detection speed and current."""
        if isinstance(self.collision_detection_speed, int):
            self.collision_detection_speed = CollisionDetectionSpeed(self.collision_detection_speed)
        if isinstance(self.collision_detection_current, int):
            self.collision_detection_current = CollisionDetectionCurrent(
                self.collision_detection_current
            )
        if isinstance(self.collision_detection_time, int):
            self.collision_detection_time = CollisionDetectionTime(self.collision_detection_time)

    @property
    def bytes(self) -> bytes:
        """Bytes representation."""
        return bytes(
            [
                self.homing_mode,
                self.homing_direction,
                *self.homing_speed.bytes,
                *self.homing_timeout.bytes,
                *self.collision_detection_speed.bytes,
                *self.collision_detection_current.bytes,
                *self.collision_detection_time.bytes,
                self.auto_home,
            ]
        )

    @property
    def data_dict(self) -> dict:
        """Dictionary representation."""
        return {
            "homing_mode": self.homing_mode.name,
            "homing_direction": self.homing_direction.name,
            f"homing_speed ({self.speed_unit.name})": self.homing_speed,
            "homing_timeout": self.homing_timeout,
            f"collision_detection_speed ({self.speed_unit.name})": self.collision_detection_speed,
            f"collision_detection_current ({self.current_unit.name})": self.collision_detection_current,  # noqa: E501
            f"collision_detection_time ({self.time_unit.name})": self.collision_detection_time,
            "auto_home": self.auto_home.name,
        }

    @classmethod
    def from_bytes(cls, data: bytes) -> "HomingParams":
        """Convert from bytes."""
        return cls(
            homing_mode=HomingMode(data[0]),
            homing_direction=HomingDirection(data[1]),
            homing_speed=HomingSpeed(to_int(data[2:4])),
            homing_timeout=HomingTimeout(to_int(data[4:8])),
            collision_detection_speed=CollisionDetectionSpeed(to_int(data[8:10])),
            collision_detection_current=CollisionDetectionCurrent(to_int(data[10:12])),
            collision_detection_time=CollisionDetectionTime(to_int(data[12:14])),
            auto_home=AutoHoming(data[14]),
        )


@dataclass
class HomingStatus(StepperOutput):
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
    def data_dict(self) -> dict[str, bool]:
        """Dictionary representation of the homing status."""
        return {
            "encoder_ready": self.encoder_ready,
            "encoder_calibrated": self.encoder_calibrated,
            "homing_active": self.is_homing,
            "homing_failed": self.homing_failed,
        }

    @classmethod
    def from_bytes(cls, data: bytes) -> "HomingStatus":
        """Convert from bytes."""
        return cls(status_code=to_int(data))


@dataclass
class VersionParams(StepperOutput):
    """Version parameters params."""

    firmware_version: int
    hardware_version: int

    @property
    def data_dict(self) -> dict:
        """Dictionary representation."""
        return {
            "firmware_version": self.firmware_version,
            "hardware_version": self.hardware_version,
        }

    @classmethod
    def from_bytes(cls, data: bytes) -> "VersionParams":
        """Convert from bytes."""
        return cls(firmware_version=data[0], hardware_version=data[1])


@dataclass
class MotorRHParams(StepperOutput):
    """Motor RH parameters params."""

    phase_resistance: int
    phase_inductance: int
    resistance_unit: ResistanceUnit = ResistanceUnit.default
    inductance_unit: InductanceUnit = InductanceUnit.default

    _r_value: float = field(init=False)
    _h_value: float = field(init=False)

    def __post_init__(self):
        """Post initialization to set resistance and inductance values."""
        self._r_value = self.phase_resistance / self.resistance_unit
        self._h_value = self.phase_inductance / self.inductance_unit

    @property
    def r_value(self) -> float:
        """Resistance value."""
        return self._r_value

    @property
    def h_value(self) -> float:
        """Inductance value."""
        return self._h_value

    @property
    def data_dict(self) -> dict:
        """Dictionary representation."""
        return {
            f"phase_resistance ({self.resistance_unit.name})": self._r_value,
            f"phase_inductance ({self.inductance_unit.name})": self._h_value,
        }

    @classmethod
    def from_bytes(cls, data: bytes) -> "MotorRHParams":
        """Convert from bytes."""
        return cls(phase_resistance=to_int(data[0:2]), phase_inductance=to_int(data[2:4]))


@dataclass
class PIDParams(StepperInput, StepperOutput):
    """PID parameters params.

    :param pid_p: PID P
    :param pid_i: PID I
    :param pid_d: PID D
    """

    pid_p: Kpid = Kpid.default_kp
    pid_i: Kpid = Kpid.default_ki
    pid_d: Kpid = Kpid.default_kd

    @property
    def bytes(self) -> bytes:
        """Bytes representation."""
        return bytes([*self.pid_p.bytes, *self.pid_i.bytes, *self.pid_d.bytes])

    @property
    def data_dict(self) -> dict:
        """Dictionary representation."""
        return {"pid_p": self.pid_p, "pid_i": self.pid_i, "pid_d": self.pid_d}

    @classmethod
    def from_bytes(cls, data: bytes) -> "PIDParams":
        """Convert from bytes."""
        return cls(
            pid_p=Kpid(to_int(data[0:4])),
            pid_i=Kpid(to_int(data[4:8])),
            pid_d=Kpid(to_int(data[8:12])),
        )


@dataclass
class BusVoltageParams(StepperOutput):
    """Bus voltage parameters params."""

    voltage: int
    unit: VoltageUnit = VoltageUnit.default

    _value: float = field(init=False)

    def __post_init__(self):
        """Post initialization to set voltage value."""
        self._value = self.voltage / self.unit.value

    @property
    def value(self) -> float:
        """Value."""
        return self._value

    @property
    def raw_value(self) -> int:
        """Raw value."""
        return self.voltage

    def data_dict(self) -> dict:
        """Dictionary representation."""
        return {f"voltage ({self.unit.name})": self._value}

    @classmethod
    def from_bytes(cls, data: bytes) -> "BusVoltageParams":
        """Convert from bytes."""
        return cls(voltage=to_int(data))


@dataclass
class PhaseCurrentParams(StepperOutput):
    """Phase current parameters params."""

    current: int
    unit: CurrentUnit = CurrentUnit.default

    _value: float = field(init=False)

    def __post_init__(self):
        """Post initialization to set current value."""
        self._value = self.current / self.unit.value

    @property
    def value(self) -> float:
        """Value."""
        return self._value

    @property
    def raw_value(self) -> int:
        """Raw value."""
        return self.current

    @property
    def data_dict(self) -> dict:
        """Dictionary representation."""
        return {f"current ({self.unit.name})": self._value}

    @classmethod
    def from_bytes(cls, data: bytes) -> "PhaseCurrentParams":
        """Convert from bytes."""
        return cls(current=to_int(data))


@dataclass
class EncoderParams(StepperOutput):
    """Encoder parameters params."""

    encoder_value: int
    unit: AngleUnit = AngleUnit.default

    _value: float = field(init=False)

    def __post_init__(self):
        """Post initialization to set angle value."""
        self._value = self.encoder_value / self.unit.value

    @property
    def value(self) -> float:
        """Value."""
        return self._value

    @property
    def angle(self) -> float:
        """Angle."""
        return self._value

    @property
    def raw_value(self) -> int:
        """Raw value."""
        return self.encoder_value

    @property
    def data_dict(self) -> dict:
        """Dictionary representation."""
        return {f"encoder_value ({self.unit.name})": self._value}

    @classmethod
    def from_bytes(cls, data: bytes) -> "EncoderParams":
        """Convert from bytes."""
        return cls(encoder_value=to_int(data))


@dataclass
class PulseCountParams(StepperOutput):
    """Pulse count parameters params."""

    pulse_count: int
    microsteps: Microstep = Microstep.default

    _value: float = field(init=False)
    _angle: float = field(init=False)

    def __post_init__(self):
        """Post initialization to set value and angle."""
        self._value = self.pulse_count / self.microsteps
        self._angle = self.pulse_count / self.microsteps / 200 * 360

    @property
    def value(self) -> float:
        """Value."""
        return self._value

    @property
    def angle(self) -> float:
        """Angle."""
        return self._angle

    @property
    def raw_value(self) -> int:
        """Raw value."""
        return self.pulse_count

    @property
    def data_dict(self) -> dict:
        """Dictionary representation."""
        return {
            "pulse_count": self.pulse_count,
            "step_count": self.pulse_count / self.microsteps,
        }

    @classmethod
    def from_bytes(cls, data: bytes) -> "PulseCountParams":
        """Convert from bytes."""
        return cls(pulse_count=to_signed_int(data))


@dataclass
class TargetPositionParams(StepperOutput):
    """Target position parameters params."""

    position: int
    unit: AngleUnit = AngleUnit.default

    _value: float = field(init=False)

    def __post_init__(self):
        """Post initialization to set value and angle."""
        self._value = self.position / self.unit.value

    @property
    def value(self) -> float:
        """Value."""
        return self._value

    @property
    def raw_value(self) -> int:
        """Raw value."""
        return self.position

    @property
    def data_dict(self) -> dict:
        """Dictionary representation."""
        return {f"target_position ({self.unit.name})": self._value}

    @classmethod
    def from_bytes(cls, data: bytes) -> "TargetPositionParams":
        """Convert from bytes."""
        return cls(position=to_signed_int(data))


@dataclass
class OpenLoopTargetPositionParams(StepperOutput):
    """Open loop target position parameters params."""

    position: int
    unit: AngleUnit = AngleUnit.default

    _value: float = field(init=False)

    def __post_init__(self):
        """Post initialization to set value."""
        self._value = self.position / self.unit.value

    @property
    def value(self) -> float:
        """Value."""
        return self._value

    @property
    def raw_value(self) -> int:
        """Raw value."""
        return self.position

    @property
    def data_dict(self) -> dict:
        """Dictionary representation."""
        return {f"open_loop_target_position ({self.unit.name})": self._value}

    @classmethod
    def from_bytes(cls, data: bytes) -> "OpenLoopTargetPositionParams":
        """Convert from bytes."""
        return cls(position=to_signed_int(data))


@dataclass
class RealTimeSpeedParams(StepperOutput):
    """Real time speed parameters params."""

    speed: int
    unit: SpeedUnit = SpeedUnit.default

    _value: float = field(init=False)

    def __post_init__(self):
        """Post initialization to set value."""
        self._value = self.speed / self.unit.value

    @property
    def value(self) -> float:
        """Value."""
        return self._value

    @property
    def raw_value(self) -> int:
        """Raw value."""
        return self.speed

    @property
    def data_dict(self) -> dict:
        """Dictionary representation."""
        return {f"real_time_speed ({self.unit.name})": self._value}

    @classmethod
    def from_bytes(cls, data: bytes) -> "RealTimeSpeedParams":
        """Convert from bytes."""
        return cls(speed=to_signed_int(data))


@dataclass
class RealTimePositionParams(StepperOutput):
    """Real time position parameters params."""

    position: int
    unit: AngleUnit = AngleUnit.default

    _value: float = field(init=False)

    def __post_init__(self):
        """Post initialization to set value."""
        self._value = self.position / self.unit.value

    @property
    def value(self) -> float:
        """Value."""
        return self._value

    @property
    def angle(self) -> float:
        """Angle."""
        return self._value

    @property
    def raw_value(self) -> int:
        """Raw value."""
        return self.position

    @property
    def data_dict(self) -> dict:
        """Dictionary representation."""
        return {f"real_time_position ({self.unit.name})": self._value}

    @classmethod
    def from_bytes(cls, data: bytes) -> "RealTimePositionParams":
        """Convert from bytes."""
        return cls(position=to_signed_int(data))


@dataclass
class PositionErrorParams(StepperOutput):
    """Position error parameters params."""

    error: int
    unit: AngleUnit = AngleUnit.default

    _value: float = field(init=False)

    def __post_init__(self):
        """Post initialization to set value."""
        self._value = self.error / self.unit.value

    @property
    def value(self) -> float:
        """Value."""
        return self._value

    @property
    def angle(self) -> float:
        """Angle."""
        return self._value

    @property
    def raw_value(self) -> int:
        """Raw value."""
        return self.error

    @property
    def data_dict(self) -> dict:
        """Dictionary representation."""
        return {f"position_error ({self.unit.name})": self._value}

    @classmethod
    def from_bytes(cls, data: bytes) -> "PositionErrorParams":
        """Convert from bytes."""
        return cls(error=to_signed_int(data))


@dataclass
class StepperStatus(StepperOutput):
    """Stepper status for read commands."""

    status_code: int
    enabled: bool = field(init=False)
    in_position: bool = field(init=False)
    stalled: bool = field(init=False)
    stall_protection_active: bool = field(init=False)

    def __post_init__(self) -> None:
        """Post initialization to set flags."""
        self.enabled = bool(self.status_code & 0x01)
        self.in_position = bool(self.status_code & 0x02)
        self.stalled = bool(self.status_code & 0x04)
        self.stall_protection_active = bool(self.status_code & 0x08)

    @property
    def data_dict(self) -> dict[str, bool]:
        """Dictionary representation of the stepper status."""
        return {
            "enabled": self.enabled,
            "in_position": self.in_position,
            "stalled": self.stalled,
            "stall_protection_active": self.stall_protection_active,
        }

    @classmethod
    def from_bytes(cls, data: bytes) -> "StepperStatus":
        """Convert from bytes."""
        return cls(status_code=to_int(data))


@dataclass
class StartSpeedParams(StepperInput):
    """Start speed parameters."""

    direction: Direction = Direction.default
    speed: Speed | int = Speed(0)
    acceleration: Acceleration | int = Acceleration(0)
    en_control: EnablePin = EnablePin.default

    def __post_init__(self):
        """Post initialization to set direction and speed."""
        if isinstance(self.speed, int):
            if self.speed < 0:
                self.direction = Direction.CCW
                self.speed = Speed(-self.speed)
            else:
                self.direction = Direction.CW
                self.speed = Speed(self.speed)
        if isinstance(self.acceleration, int):
            self.acceleration = Acceleration(self.acceleration)

    @property
    def bytes(self) -> bytes:
        """Bytes representation."""
        return bytes(
            [
                self.direction,
                *self.speed.bytes,
                self.acceleration,
                self.en_control,
            ]
        )

    @property
    def data_dict(self) -> dict:
        """Dictionary representation."""
        return {
            "direction": self.direction.name,
            "speed": self.speed,
            "acceleration": self.acceleration,
            "enable_control": self.en_control.name,
        }


@dataclass
class ConfigParams(StepperInput, StepperOutput):
    """Motor parameters params.

    :param stepper_type: Motor type
    :param control_mode: Control mode
    :param communication_mode: Communication mode
    :param enable_level: Enable level
    :param default_direction: Default direction
    :param microsteps: Microsteps
    :param microstep_interp: Microstep interpolation
    :param screen_off: Screen off
    :param open_loop_current: Open loop current
    :param max_closed_loop_current: Maximum closed loop current
    :param max_voltage: Maximum voltage
    :param baud_rate: Baud rate
    :param can_rate: CAN rate
    :param address: Device ID
    :param checksum_mode: Checksum mode
    :param response_mode: Response mode
    :param stall_protect: Stall protection
    :param stall_speed: Stall speed
    :param stall_current: Stall current
    :param stall_time: Stall time
    :param pos_window: Position window
    """

    stepper_type: MotorType
    control_mode: ControlMode
    communication_mode: CommunicationMode
    enable_level: EnableLevel
    default_direction: Direction
    microsteps: Microstep
    microstep_interp: MicrostepInterp
    screen_off: ScreenOff
    open_loop_current: OpenLoopCurrent
    max_closed_loop_current: ClosedLoopCurrent
    max_voltage: MaxVoltage
    baud_rate: BaudRate
    can_rate: CanRate
    address: Address
    checksum_mode: ChecksumMode
    response_mode: ResponseMode
    stall_protect: StallProtect
    stall_speed: StallSpeed
    stall_current: StallCurrent
    stall_time: StallTime
    on_target_window: OnTargetWindow

    current_unit: CurrentUnit = CurrentUnit.default
    voltage_unit: VoltageUnit = VoltageUnit.default
    angle_unit: AngleUnit = AngleUnit.default
    time_unit: TimeUnit = TimeUnit.default
    speed_unit: SpeedUnit = SpeedUnit.default

    _open_loop_current: float = field(init=False)
    _max_closed_loop_current: float = field(init=False)
    _max_voltage: float = field(init=False)
    _stall_speed: float = field(init=False)
    _stall_current: float = field(init=False)
    _stall_time: float = field(init=False)
    _on_target_window: float = field(init=False)

    def __post_init__(self):
        """Post initialization to set direction and speed."""
        self.microsteps = Microstep(self.microsteps)
        self.open_loop_current = OpenLoopCurrent(self.open_loop_current)
        self.max_closed_loop_current = ClosedLoopCurrent(self.max_closed_loop_current)
        self.max_voltage = MaxVoltage(self.max_voltage)
        self.stall_speed = StallSpeed(self.stall_speed)
        self.stall_current = StallCurrent(self.stall_current)
        self.stall_time = StallTime(self.stall_time)
        if isinstance(self.on_target_window, int):
            self.on_target_window = OnTargetWindow(self.on_target_window)
        elif isinstance(self.on_target_window, float):
            self.on_target_window = OnTargetWindow(int(self.on_target_window * 10))

        self._open_loop_current = self.open_loop_current / self.current_unit
        self._max_closed_loop_current = self.max_closed_loop_current / self.current_unit
        self._max_voltage = self.max_voltage / self.voltage_unit
        self._stall_speed = self.stall_speed / self.speed_unit
        self._stall_current = self.stall_current / self.current_unit
        self._stall_time = self.stall_time / self.time_unit
        self._on_target_window = self.on_target_window / 10

    @property
    def open_loop_current_value(self) -> float:
        """Open loop current value."""
        return self._open_loop_current

    @property
    def max_closed_loop_current_value(self) -> float:
        """Max closed loop current value."""
        return self._max_closed_loop_current

    @property
    def max_voltage_value(self) -> float:
        """Max voltage value."""
        return self._max_voltage

    @property
    def stall_speed_value(self) -> float:
        """Stall speed value."""
        return self._stall_speed

    @property
    def stall_current_value(self) -> float:
        """Stall current value."""
        return self._stall_current

    @property
    def stall_time_value(self) -> float:
        """Stall time value."""
        return self._stall_time

    @property
    def on_target_window_value(self) -> float:
        """On target window value."""
        return self._on_target_window

    @property
    def bytes(self) -> bytes:
        """Bytes representation."""
        return bytes(
            [
                self.stepper_type,
                self.control_mode,
                self.communication_mode,
                self.enable_level,
                self.default_direction,
                self.microsteps,
                self.microstep_interp,
                self.screen_off,
                *self.open_loop_current.bytes,
                *self.max_closed_loop_current.bytes,
                *self.max_voltage.bytes,
                self.baud_rate,
                self.can_rate,
                self.address,
                self.checksum_mode,
                self.response_mode,
                self.stall_protect,
                *self.stall_speed.bytes,
                *self.stall_current.bytes,
                *self.stall_time.bytes,
                *self.on_target_window.bytes,
            ]
        )

    @property
    def data_dict(self) -> dict:
        """Response dictionary."""
        return {
            "stepper_type": self.stepper_type.name,
            "control_mode": self.control_mode.name,
            "communication_mode": self.communication_mode.name,
            "enable_level": self.enable_level.name,
            "default_direction": self.default_direction.name,
            "microsteps": self.microsteps,
            "microstep_interp": self.microstep_interp.name,
            "screen_off": self.screen_off.name,
            f"open_loop_current ({self.current_unit.name})": self._open_loop_current,
            f"max_closed_loop_current ({self.current_unit.name})": self._max_closed_loop_current,
            f"max_voltage ({self.voltage_unit.name})": self._max_voltage,
            "baud_rate": self.baud_rate.name,
            "can_rate": self.can_rate.name,
            "address": self.address,
            "checksum_mode": self.checksum_mode.name,
            "response_mode": self.response_mode.name,
            "stall_protect": self.stall_protect.name,
            f"stall_speed ({self.speed_unit.name})": self._stall_speed,
            f"stall_current ({self.current_unit.name})": self._stall_current,
            f"stall_time ({self.time_unit.name})": self._stall_time,
            "on_target_window": self._on_target_window,
        }

    @classmethod
    def from_bytes(cls, data: bytes) -> "ConfigParams":
        """Convert from bytes."""
        if len(data) != 30:
            raise ValueError(f"Invalid data length {len(data)}")
        return cls(
            stepper_type=MotorType(data[2]),
            control_mode=ControlMode(data[3]),
            communication_mode=CommunicationMode(data[4]),
            enable_level=EnableLevel(data[5]),
            default_direction=Direction(data[6]),
            microsteps=Microstep(data[7]),
            microstep_interp=MicrostepInterp(data[8]),
            screen_off=ScreenOff(data[9]),
            open_loop_current=OpenLoopCurrent(to_int(data[10:12])),
            max_closed_loop_current=ClosedLoopCurrent(to_int(data[12:14])),
            max_voltage=MaxVoltage(to_int(data[14:16])),
            baud_rate=BaudRate(data[16]),
            can_rate=CanRate(data[17]),
            address=Address(data[18]),
            checksum_mode=ChecksumMode(data[19]),
            response_mode=ResponseMode(data[20]),
            stall_protect=StallProtect(data[21]),
            stall_speed=StallSpeed(to_int(data[22:24])),
            stall_current=StallCurrent(to_int(data[24:26])),
            stall_time=StallTime(to_int(data[26:28])),
            on_target_window=OnTargetWindow(to_int(data[28:30])),
        )


@dataclass
class SystemParams(StepperOutput):
    """System parameters params.

    :param voltage_unit: Voltage unit
    :param current_unit: Current unit
    :param angle_unit: Angle unit
    :param bus_voltage: Bus voltage
    :param bus_phase_current: Bus phase current
    :param calibrated_encoder_value: Calibrated encoder value
    :param stepper_target_position: Motor target position
    :param stepper_real_time_speed: Motor real time speed
    :param stepper_real_time_position: Motor real time position
    :param stepper_position_error: Motor position error
    :param homing_encoder_ready: Homing encoder ready
    :param homing_encoder_calibrated: Homing encoder calibrated
    :param homing_active: Homing active
    :param homing_failed: Homing failed
    :param stepper_enabled: Stepper enabled
    :param stepper_in_position: Stepper in position
    :param stepper_stalled: Stepper stalled
    :param stepper_stall_protection_active: Stepper stall protection active
    """

    bus_voltage: int
    bus_phase_current: int
    calibrated_encoder_value: int
    stepper_target_position: int
    stepper_real_time_speed: int
    stepper_real_time_position: int
    stepper_position_error: int
    homing_status: HomingStatus
    stepper_status: StepperStatus

    voltage_unit: VoltageUnit = VoltageUnit.default
    current_unit: CurrentUnit = CurrentUnit.default
    angle_unit: AngleUnit = AngleUnit.default
    speed_unit: SpeedUnit = SpeedUnit.default

    _bus_voltage: float = field(init=False)
    _bus_phase_current: float = field(init=False)
    _calibrated_encoder_value: float = field(init=False)
    _stepper_target_position: float = field(init=False)
    _stepper_real_time_speed: float = field(init=False)
    _stepper_real_time_position: float = field(init=False)
    _stepper_position_error: float = field(init=False)

    def __post_init__(self):
        """Post initialization to set values."""
        self._bus_voltage = self.bus_voltage / self.voltage_unit
        self._bus_phase_current = self.bus_phase_current / self.current_unit
        self._calibrated_encoder_value = self.calibrated_encoder_value / self.angle_unit.value
        self._stepper_target_position = self.stepper_target_position / self.angle_unit.value
        self._stepper_real_time_speed = self.stepper_real_time_speed / self.speed_unit
        self._stepper_real_time_position = self.stepper_real_time_position / self.angle_unit.value
        self._stepper_position_error = self.stepper_position_error / self.angle_unit.value

    @property
    def bus_voltage_value(self) -> float:
        """Bus voltage value."""
        return self._bus_voltage

    @property
    def bus_phase_current_value(self) -> float:
        """Bus phase current value."""
        return self._bus_phase_current

    @property
    def calibrated_encoder_angle(self) -> float:
        """Calibrated encoder value value."""
        return self._calibrated_encoder_value

    @property
    def stepper_target_angle(self) -> float:
        """Stepper target position value."""
        return self._stepper_target_position

    @property
    def stepper_real_time_speed_value(self) -> float:
        """Stepper real time speed value."""
        return self._stepper_real_time_speed

    @property
    def stepper_real_time_position_angle(self) -> float:
        """Stepper real time position value."""
        return self._stepper_real_time_position

    @property
    def stepper_position_error_angle(self) -> float:
        """Stepper position error value."""
        return self._stepper_position_error

    @property
    def data_dict(self) -> dict:
        """Dictionary representation."""
        return {
            f"bus_voltage ({self.voltage_unit.name})": self._bus_voltage,
            f"bus_phase_current ({self.current_unit.name})": self._bus_phase_current,
            f"calibrated_encoder_value ({self.angle_unit.name})": self._calibrated_encoder_value,
            f"stepper_target_position ({self.angle_unit.name})": self._stepper_target_position,
            f"stepper_real_time_speed ({self.speed_unit.name})": self._stepper_real_time_speed,
            f"stepper_real_time_position ({self.angle_unit.name})": self._stepper_real_time_position,  # noqa: E501
            f"stepper_position_error ({self.angle_unit.name})": self._stepper_position_error,
            "encoder_ready": self.homing_status.encoder_ready,
            "encoder_calibrated": self.homing_status.encoder_calibrated,
            "is_homing": self.homing_status.is_homing,
            "homing_failed": self.homing_status.homing_failed,
            "stepper_enabled": self.stepper_status.enabled,
            "stepper_in_position": self.stepper_status.in_position,
            "stepper_stalled": self.stepper_status.stalled,
            "stepper_stall_protection_active": self.stepper_status.stall_protection_active,
        }

    @classmethod
    def from_bytes(cls, data: bytes) -> "SystemParams":
        """Convert from bytes."""
        if len(data) != 28:
            raise ValueError("Invalid data length")
        return cls(
            bus_voltage=to_int(data[2:4]),
            bus_phase_current=to_int(data[4:6]),
            calibrated_encoder_value=to_int(data[6:8]),
            stepper_target_position=to_signed_int(data[8:13]),
            stepper_real_time_speed=to_signed_int(data[13:16]),
            stepper_real_time_position=to_signed_int(data[16:21]),
            stepper_position_error=to_signed_int(data[21:26]),
            homing_status=HomingStatus(data[26]),
            stepper_status=StepperStatus(data[27]),
        )


@dataclass
class InputParams:
    """Parameters that can only be used as input.

    :param jog_params: Parameters for jog movement
    :param position_params: Parameters for position movement
    :param start_speed_params: Parameters for start speed configuration
    :param loop_mode: Control loop mode (open/closed)
    :param speed_reduction: Speed reduction mode
    :param sync_flag: Sync flag for movement
    :param store_flag: Store flag for settings
    """

    jog_params: JogParams = field(default_factory=JogParams)
    position_params: PositionParams = field(default_factory=PositionParams)
    start_speed_params: StartSpeedParams = field(default_factory=StartSpeedParams)
    loop_mode: LoopMode = LoopMode.default
    speed_reduction: SpeedReduction = SpeedReduction.default
    sync_flag: SyncFlag = SyncFlag.default
    store_flag: StoreFlag = StoreFlag.default

    def __post_init__(self) -> None:
        """Validate enum parameters."""
        if isinstance(self.loop_mode, int):
            self.loop_mode = LoopMode(self.loop_mode)
        if isinstance(self.speed_reduction, int):
            self.speed_reduction = SpeedReduction(self.speed_reduction)
        if isinstance(self.sync_flag, int):
            self.sync_flag = SyncFlag(self.sync_flag)
        if isinstance(self.store_flag, int):
            self.store_flag = StoreFlag(self.store_flag)


@dataclass
class OutputParams:
    """Parameters that can only be used as output."""

    version_params: VersionParams
    motor_rh_params: MotorRHParams
    bus_voltage_params: BusVoltageParams
    phase_current_params: PhaseCurrentParams
    encoder_params: EncoderParams
    pulse_count_params: PulseCountParams
    target_position_params: TargetPositionParams
    open_loop_target_position_params: OpenLoopTargetPositionParams
    real_time_speed_params: RealTimeSpeedParams
    real_time_position_params: RealTimePositionParams
    position_error_params: PositionErrorParams
    stepper_status: StepperStatus
    system_params: SystemParams
    homing_status: HomingStatus


@dataclass
class InputOutputParams:
    """Parameters that can be used as both input and output."""

    homing_params: HomingParams
    pid_params: PIDParams
    config_params: ConfigParams
