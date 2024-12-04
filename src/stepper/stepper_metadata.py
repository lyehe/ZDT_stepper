"""Metadata container classes."""

import json
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import TypeAlias

import yaml

from .stepper_constants import (
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
    HomingDirection,
    HomingMode,
    HomingSpeed,
    HomingTimeout,
    Kpid,
    MaxVoltage,
    Microstep,
    MicrostepInterp,
    MotorType,
    OnTargetWindow,
    OpenLoopCurrent,
    PulseCount,
    ResponseMode,
    ScreenOff,
    Speed,
    StallCurrent,
    StallProtect,
    StallSpeed,
    StallTime,
    StoreFlag,
    SyncFlag,
    VoltageUnit,
)

PathVar: TypeAlias = Path | str


@dataclass
class StepperMetadata:
    """Metadata class for stepper metadata."""

    def to_dict(self) -> dict:
        """Convert to dictionary."""
        return asdict(self)

    @classmethod
    def from_dict(cls, data: dict) -> "StepperMetadata":
        """Convert from dictionary."""
        return cls(**data)

    def to_json(self, path: PathVar) -> None:
        """Convert to JSON."""
        with open(path, "w") as f:
            json.dump(self.to_dict(), f, indent=2)

    @classmethod
    def from_json(cls, path: PathVar) -> "StepperMetadata":
        """Convert from JSON."""
        with open(path) as f:
            return cls.from_dict(json.load(f))

    def to_yaml(self, path: PathVar) -> None:
        """Convert to YAML."""
        with open(path, "w") as f:
            yaml.dump(self.to_dict(), f)

    @classmethod
    def from_yaml(cls, path: PathVar) -> "StepperMetadata":
        """Convert from YAML."""
        with open(path) as f:
            return cls.from_dict(yaml.safe_load(f))


@dataclass
class PositionData(StepperMetadata):
    """Position data metadata."""

    direction: Direction = Direction.default
    speed: Speed = Speed.default
    acceleration: Acceleration = Acceleration.default
    pulse_count: PulseCount = PulseCount.default
    absolute: AbsoluteFlag = AbsoluteFlag.default
    sync: SyncFlag = SyncFlag.default


@dataclass
class VelocityData(StepperMetadata):
    """Velocity data metadata."""

    direction: Direction = Direction.default
    speed: Speed = Speed.default
    acceleration: Acceleration = Acceleration.default
    sync: SyncFlag = SyncFlag.default


@dataclass
class HomeMetadata(StepperMetadata):
    """Home parameters metadata.

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

    store: StoreFlag
    homing_mode: HomingMode
    homing_direction: HomingDirection
    homing_speed: HomingSpeed
    homing_timeout: HomingTimeout
    collision_detection_speed: CollisionDetectionSpeed
    collision_detection_current: CollisionDetectionCurrent
    collision_detection_time: CollisionDetectionTime
    auto_home: AutoHoming


@dataclass
class MotorMetadata(StepperMetadata):
    """Motor parameters metadata.

    :param motor_type: Motor type
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
    :param device_id: Device ID
    :param checksum_mode: Checksum mode
    :param response_mode: Response mode
    :param stall_protect: Stall protection
    :param stall_speed: Stall speed
    :param stall_current: Stall current
    :param stall_time: Stall time
    :param pos_window: Position window
    """

    motor_type: MotorType = MotorType.default
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
    device_id: Address = Address.default
    checksum_mode: ChecksumMode = ChecksumMode.default
    response_mode: ResponseMode = ResponseMode.default
    stall_protect: StallProtect = StallProtect.default
    stall_speed: StallSpeed = StallSpeed.default
    stall_current: StallCurrent = StallCurrent.default
    stall_time: StallTime = StallTime.default
    on_target_window: OnTargetWindow = OnTargetWindow.default


@dataclass
class SystemMetadata(StepperMetadata):
    """System parameters metadata.

    :param voltage_unit: Voltage unit
    :param current_unit: Current unit
    :param angle_unit: Angle unit
    :param bus_voltage: Bus voltage
    :param bus_phase_current: Bus phase current
    :param calibrated_encoder_value: Calibrated encoder value
    :param motor_target_position: Motor target position
    :param motor_real_time_speed: Motor real time speed
    :param motor_real_time_position: Motor real time position
    :param motor_position_error: Motor position error
    :param homing_encoder_ready: Homing encoder ready
    :param homing_encoder_calibrated: Homing encoder calibrated
    :param homing_active: Homing active
    :param homing_failed: Homing failed
    :param stepper_enabled: Stepper enabled
    :param stepper_in_position: Stepper in position
    :param stepper_stalled: Stepper stalled
    :param stepper_stall_protection_active: Stepper stall protection active
    """

    voltage_unit: VoltageUnit
    current_unit: CurrentUnit
    angle_unit: AngleUnit
    bus_voltage: float
    bus_phase_current: float
    calibrated_encoder_value: float
    motor_target_position: float
    motor_real_time_speed: float
    motor_real_time_position: float
    motor_position_error: float
    homing_encoder_ready: bool
    homing_encoder_calibrated: bool
    homing_active: bool
    homing_failed: bool
    stepper_enabled: bool
    stepper_in_position: bool
    stepper_stalled: bool
    stepper_stall_protection_active: bool


@dataclass
class PIDMetadata(StepperMetadata):
    """PID parameters metadata.

    :param pid_p: PID P
    :param pid_i: PID I
    :param pid_d: PID D
    """

    pid_p: Kpid = Kpid.default_kp
    pid_i: Kpid = Kpid.default_ki
    pid_d: Kpid = Kpid.default_kd


@dataclass
class ReadableMetadata:
    """Readable metadata.

    :param system_metadata: System metadata parameters
    :param motor_metadata: Motor metadata parameters
    :param home_metadata: Home metadata parameters
    :param pid_metadata: PID metadata parameters
    """

    system_metadata: SystemMetadata | None = None
    motor_metadata: MotorMetadata | None = None
    home_metadata: HomeMetadata | None = None
    pid_metadata: PIDMetadata | None = None

    @property
    def __dict__(self) -> dict:
        """Dictionary representation."""
        result = {}
        if self.system_metadata is not None:
            result.update(asdict(self.system_metadata))
        if self.motor_metadata is not None:
            result.update(asdict(self.motor_metadata))
        if self.home_metadata is not None:
            result.update(asdict(self.home_metadata))
        if self.pid_metadata is not None:
            result.update(asdict(self.pid_metadata))
        return result


@dataclass
class WritableMetadata:
    """Writable metadata."""

    position_metadata: PositionData | None = None
    velocity_metadata: VelocityData | None = None
    home_metadata: HomeMetadata | None = None
    motor_metadata: MotorMetadata | None = None
    pid_metadata: PIDMetadata | None = None

    @property
    def __dict__(self) -> dict:
        """Dictionary representation."""
        result = {}
        if self.position_metadata is not None:
            result.update(asdict(self.position_metadata))
        if self.velocity_metadata is not None:
            result.update(asdict(self.velocity_metadata))
        if self.home_metadata is not None:
            result.update(asdict(self.home_metadata))
        if self.motor_metadata is not None:
            result.update(asdict(self.motor_metadata))
        if self.pid_metadata is not None:
            result.update(asdict(self.pid_metadata))
        return result
