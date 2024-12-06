"""params container classes."""

import json
from abc import ABC, abstractmethod
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import TypeAlias

import yaml
from serial import Serial

from .constants import (
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
    SpeedUnit,
    StallCurrent,
    StallProtect,
    StallSpeed,
    StallTime,
    TimeUnit,
    VoltageUnit,
)

PathVar: TypeAlias = Path | str


def to_int(input: bytes) -> int:
    """Convert bytes to int."""
    return int.from_bytes(input, "big")


def to_signed_int(input: bytes) -> int:
    """Convert long bytes to signed int."""
    sign = -1 if input[0] == 1 else 1
    return sign * to_int(input[1:])


@dataclass
class DeviceParams:
    """Device parameter class.

    :param serial_connection: Serial connection object
    :param address: Device address
    :param checksum_mode: Checksum mode
    :param delay: Communication delay in seconds
    """

    serial_connection: Serial
    address: Address = Address.default
    checksum_mode: ChecksumMode = ChecksumMode.default
    delay: float | None = None


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
    speed: Speed = Speed.default
    acceleration: Acceleration = Acceleration.default

    @property
    def bytes(self) -> bytes:
        """Bytes representation."""
        return bytes([self.direction, *self.speed.bytes, self.acceleration])


@dataclass
class PositionParams(StepperInput):
    """Position data params."""

    direction: Direction = Direction.default
    speed: Speed = Speed.default
    acceleration: Acceleration = Acceleration.default
    pulse_count: PulseCount = PulseCount.default
    absolute: AbsoluteFlag = AbsoluteFlag.default

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
    collision_detection_speed: CollisionDetectionSpeed
    collision_detection_current: CollisionDetectionCurrent
    collision_detection_time: CollisionDetectionTime
    auto_home: AutoHoming

    current_unit: CurrentUnit = CurrentUnit.default
    time_unit: TimeUnit = TimeUnit.default
    speed_unit: SpeedUnit = SpeedUnit.default

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
            f"homing_speed ({self.speed_unit.name})": self.homing_speed / self.speed_unit,
            "homing_timeout": self.homing_timeout,
            f"collision_detection_speed ({self.speed_unit.name})": self.collision_detection_speed
            / self.speed_unit,
            f"collision_detection_current ({self.current_unit.name})": self.collision_detection_current  # noqa: E501
            / self.current_unit,
            f"collision_detection_time ({self.time_unit.name})": self.collision_detection_time
            / self.time_unit,
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

    @property
    def data_dict(self) -> dict:
        """Dictionary representation."""
        return {
            f"phase_resistance ({self.resistance_unit.name})": self.phase_resistance
            / self.resistance_unit,
            f"phase_inductance ({self.inductance_unit.name})": self.phase_inductance
            / self.inductance_unit,
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

    @property
    def data_dict(self) -> dict:
        """Dictionary representation."""
        return {f"voltage ({self.unit.name})": self.voltage / self.unit.value}

    @classmethod
    def from_bytes(cls, data: bytes) -> "BusVoltageParams":
        """Convert from bytes."""
        return cls(voltage=to_int(data))


@dataclass
class PhaseCurrentParams(StepperOutput):
    """Phase current parameters params."""

    current: int
    unit: CurrentUnit = CurrentUnit.default

    @property
    def data_dict(self) -> dict:
        """Dictionary representation."""
        return {f"current ({self.unit.name})": self.current / self.unit.value}

    @classmethod
    def from_bytes(cls, data: bytes) -> "PhaseCurrentParams":
        """Convert from bytes."""
        return cls(current=to_int(data))


@dataclass
class EncoderParams(StepperOutput):
    """Encoder parameters params."""

    encoder_value: int
    unit: AngleUnit = AngleUnit.default

    @property
    def data_dict(self) -> dict:
        """Dictionary representation."""
        return {f"encoder_value ({self.unit.name})": self.encoder_value / self.unit.value}

    @classmethod
    def from_bytes(cls, data: bytes) -> "EncoderParams":
        """Convert from bytes."""
        return cls(encoder_value=to_int(data))


@dataclass
class PulseCountParams(StepperOutput):
    """Pulse count parameters params."""

    pulse_count: int
    microsteps: Microstep = Microstep.default

    @property
    def data_dict(self) -> dict:
        """Dictionary representation."""
        return {"pulse_count": self.pulse_count, "step_count": self.pulse_count / self.microsteps}

    @classmethod
    def from_bytes(cls, data: bytes) -> "PulseCountParams":
        """Convert from bytes."""
        return cls(pulse_count=to_signed_int(data))


@dataclass
class TargetPositionParams(StepperOutput):
    """Target position parameters params."""

    position: int
    unit: AngleUnit = AngleUnit.default

    @property
    def data_dict(self) -> dict:
        """Dictionary representation."""
        return {f"target_position ({self.unit.name})": self.position / self.unit.value}

    @classmethod
    def from_bytes(cls, data: bytes) -> "TargetPositionParams":
        """Convert from bytes."""
        return cls(position=to_signed_int(data))


@dataclass
class OpenLoopTargetPositionParams(StepperOutput):
    """Open loop target position parameters params."""

    position: int
    unit: AngleUnit = AngleUnit.default

    @property
    def data_dict(self) -> dict:
        """Dictionary representation."""
        return {f"open_loop_target_position ({self.unit.name})": self.position / self.unit.value}

    @classmethod
    def from_bytes(cls, data: bytes) -> "OpenLoopTargetPositionParams":
        """Convert from bytes."""
        return cls(position=to_signed_int(data))


@dataclass
class RealTimeSpeedParams(StepperOutput):
    """Real time speed parameters params."""

    speed: int
    unit: SpeedUnit = SpeedUnit.default

    @property
    def data_dict(self) -> dict:
        """Dictionary representation."""
        return {f"real_time_speed ({self.unit.name})": self.speed / self.unit}

    @classmethod
    def from_bytes(cls, data: bytes) -> "RealTimeSpeedParams":
        """Convert from bytes."""
        return cls(speed=to_signed_int(data))


@dataclass
class RealTimePositionParams(StepperOutput):
    """Real time position parameters params."""

    position: int
    unit: AngleUnit = AngleUnit.default

    @property
    def data_dict(self) -> dict:
        """Dictionary representation."""
        return {f"real_time_position ({self.unit.name})": self.position / self.unit.value}

    @classmethod
    def from_bytes(cls, data: bytes) -> "RealTimePositionParams":
        """Convert from bytes."""
        return cls(position=to_signed_int(data))


@dataclass
class PositionErrorParams(StepperOutput):
    """Position error parameters params."""

    error: int
    unit: AngleUnit = AngleUnit.default

    @property
    def data_dict(self) -> dict:
        """Dictionary representation."""
        return {f"position_error ({self.unit.name})": self.error / self.unit.value}

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
    speed: Speed = Speed.default
    acceleration: Acceleration = Acceleration.default
    en_control: EnablePin = EnablePin.default

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

    stepper_type: MotorType = MotorType.default
    control_mode: ControlMode = ControlMode.default
    communication_mode: CommunicationMode = CommunicationMode.default
    enable_level: EnableLevel = EnableLevel.default
    default_direction: Direction = Direction.default
    microsteps: Microstep = Microstep.default
    microstep_interp: MicrostepInterp = MicrostepInterp.default
    screen_off: ScreenOff = ScreenOff.default
    open_loop_current: OpenLoopCurrent = OpenLoopCurrent.default
    max_closed_loop_current: ClosedLoopCurrent = ClosedLoopCurrent.default
    max_voltage: MaxVoltage = MaxVoltage.default
    baud_rate: BaudRate = BaudRate.default
    can_rate: CanRate = CanRate.default
    address: Address = Address.default
    checksum_mode: ChecksumMode = ChecksumMode.default
    response_mode: ResponseMode = ResponseMode.default
    stall_protect: StallProtect = StallProtect.default
    stall_speed: StallSpeed = StallSpeed.default
    stall_current: StallCurrent = StallCurrent.default
    stall_time: StallTime = StallTime.default
    on_target_window: OnTargetWindow = OnTargetWindow.default

    current_unit: CurrentUnit = CurrentUnit.default
    voltage_unit: VoltageUnit = VoltageUnit.default
    angle_unit: AngleUnit = AngleUnit.default
    time_unit: TimeUnit = TimeUnit.default
    speed_unit: SpeedUnit = SpeedUnit.default

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
            f"open_loop_current ({self.current_unit.name})": self.open_loop_current
            / self.current_unit,
            f"max_closed_loop_current ({self.current_unit.name})": self.max_closed_loop_current
            / self.current_unit,
            f"max_voltage ({self.voltage_unit.name})": self.max_voltage / self.voltage_unit,
            "baud_rate": self.baud_rate.name,
            "can_rate": self.can_rate.name,
            "address": self.address,
            "checksum_mode": self.checksum_mode.name,
            "response_mode": self.response_mode.name,
            "stall_protect": self.stall_protect.name,
            f"stall_speed ({self.speed_unit.name})": self.stall_speed / self.speed_unit,
            f"stall_current ({self.current_unit.name})": self.stall_current / self.current_unit,
            f"stall_time ({self.time_unit.name})": self.stall_time / self.time_unit,
            "on_target_window": self.on_target_window / 10,
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

    @property
    def data_dict(self) -> dict:
        """Dictionary representation."""
        return {
            f"bus_voltage ({self.voltage_unit.name})": self.bus_voltage / self.voltage_unit,
            f"bus_phase_current ({self.current_unit.name})": self.bus_phase_current
            / self.current_unit,
            f"calibrated_encoder_value ({self.angle_unit.name})": self.calibrated_encoder_value
            / self.angle_unit.value,
            f"stepper_target_position ({self.angle_unit.name})": self.stepper_target_position
            / self.angle_unit.value,
            f"stepper_real_time_speed ({self.speed_unit.name})": self.stepper_real_time_speed
            / self.speed_unit,
            f"stepper_real_time_position ({self.angle_unit.name})": self.stepper_real_time_position
            / self.angle_unit.value,
            f"stepper_position_error ({self.angle_unit.name})": self.stepper_position_error
            / self.angle_unit.value,
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
class Readables:
    """Readable params.

    :param system_params: System params parameters
    :param stepper_params: Motor params parameters
    :param home_params: Home params parameters
    :param pid_params: PID params parameters
    """

    system_params: SystemParams | None = None
    stepper_params: ConfigParams | None = None
    home_params: HomingParams | None = None
    pid_params: PIDParams | None = None

    @property
    def data_dict(self) -> dict:
        """Dictionary representation."""
        result = {}
        if self.system_params is not None:
            result.update(asdict(self.system_params))
        if self.stepper_params is not None:
            result.update(asdict(self.stepper_params))
        if self.home_params is not None:
            result.update(asdict(self.home_params))
        if self.pid_params is not None:
            result.update(asdict(self.pid_params))
        return result


@dataclass
class Writables:
    """Writable params."""

    position_params: PositionParams | None = None
    velocity_params: JogParams | None = None
    home_params: HomingParams | None = None
    stepper_params: ConfigParams | None = None
    pid_params: PIDParams | None = None

    @property
    def data_dict(self) -> dict:
        """Dictionary representation."""
        result = {}
        if self.position_params is not None:
            result.update(asdict(self.position_params))
        if self.velocity_params is not None:
            result.update(asdict(self.velocity_params))
        if self.home_params is not None:
            result.update(asdict(self.home_params))
        if self.stepper_params is not None:
            result.update(asdict(self.stepper_params))
        if self.pid_params is not None:
            result.update(asdict(self.pid_params))
        return result
