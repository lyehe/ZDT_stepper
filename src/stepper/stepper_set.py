from abc import abstractmethod
from dataclasses import dataclass
from logging import getLogger

from .stepper_command import BroadcastCommand, Command
from .stepper_constants import (
    Acceleration,
    Address,
    BaudRate,
    CanRate,
    ChecksumMode,
    ClosedLoopCurrent,
    Code,
    CommunicationMode,
    ControlMode,
    CurrentUnit,
    DefaultDir,
    Direction,
    EnableLevel,
    EnablePin,
    Kpid,
    LoopMode,
    MaxVoltage,
    Microstep,
    MicrostepInterp,
    MotorType,
    OnTargetWindow,
    OpenLoopCurrent,
    Protocol,
    ResponseMode,
    ScreenOff,
    Speed,
    SpeedReduction,
    StallProtect,
    StallSpeed,
    StallTime,
    StoreFlag,
)
from .stepper_exceptions import CommandError

logger = getLogger(__name__)


@abstractmethod
class SetCommand(Command):
    """Set command configuration."""


@dataclass
class SetMicrostep(SetCommand):
    """Set microstep command configuration.

    :param store: Store flag
    :param microstep_value: Microstep value (0-255, 0 is 256 microsteps)
    """

    store: StoreFlag = StoreFlag.default
    microstep_value: Microstep = Microstep.default

    @property
    def _code(self) -> Code:
        return Code.SET_MICROSTEP

    @property
    def _input(self) -> bytes:
        return bytes(
            [
                self.addr,
                self._code,
                Protocol.SET_MICROSTEP,
                self.store,
                self.microstep_value,
            ]
        )


@dataclass
class SetMicrostepAll(SetMicrostep, BroadcastCommand):
    """Set microstep for all motors."""


@dataclass
class SetID(SetCommand):
    """Set ID.

    :param store: Store flag
    :param device_id: Device ID
    :param confirm: Confirm flag, defaults to False
    """

    store: StoreFlag = StoreFlag.default
    device_id: Address = Address.default
    confirm: bool = False

    @property
    def _code(self) -> Code:
        return Code.SET_ID

    @property
    def _input(self) -> bytes:
        if self.confirm:
            return bytes([self.addr, self._code, Protocol.SET_ID, self.store, self.device_id])
        else:
            logger.warning(
                "The device will be set to a different device ID. Please confirm by setting the confirm to true."
            )
            raise CommandError("Confirm flag must be set to True to confirm ID change")


@dataclass
class SetLoopMode(SetCommand):
    """Set loop mode command configuration.

    :param store: Store flag
    :param control_mode: Control mode
    """

    store: StoreFlag = StoreFlag.default
    loop_mode: LoopMode = LoopMode.default

    @property
    def _code(self) -> Code:
        return Code.SET_LOOP_MODE

    @property
    def _input(self) -> bytes:
        return bytes([self.addr, self._code, Protocol.SET_LOOP_MODE, self.store, self.loop_mode])


@dataclass
class SetLoopModeAll(SetLoopMode, BroadcastCommand):
    """Set loop mode for all motors."""


@dataclass
class SetOpenLoopCurrent(SetCommand):
    """Set open loop current.
    
    :param store: Store flag
    :param open_loop_current: Open loop current
    """

    store: StoreFlag
    open_loop_current: OpenLoopCurrent
    current_unit: CurrentUnit = CurrentUnit.MA

    @property
    def _code(self) -> Code:
        return Code.SET_OPEN_LOOP_CURRENT

    @property
    def _input(self) -> bytes:
        return bytes(
            [
                self.addr,
                self._code,
                Protocol.SET_OPEN_LOOP_CURRENT,
                self.store,
                *self.open_loop_current.bytes,
            ]
        )


@dataclass
class SetOpenLoopCurrentAll(SetOpenLoopCurrent, BroadcastCommand):
    """Set open loop current for all motors command configuration"""


@dataclass
class SetPID(SetCommand):
    """Set PID parameters command configuration

    :param store: Store flag
    :param kp: Proportional gain
    :param ki: Integral gain
    :param kd: Derivative gain
    """

    store: StoreFlag = StoreFlag.default
    kp: Kpid = Kpid.default_kp
    ki: Kpid = Kpid.default_ki
    kd: Kpid = Kpid.default_kd

    @property
    def _code(self) -> Code:
        return Code.SET_PID

    @property
    def _input(self) -> bytes:
        return bytes(
            [
                self.addr,
                self._code,
                Protocol.SET_PID,
                self.store,
                *self.kp.bytes,
                *self.ki.bytes,
                *self.kd.bytes,
            ]
        )


@dataclass
class SetPIDAll(SetPID, BroadcastCommand):
    """Set PID parameters for all motors command configuration."""


@dataclass
class SetStartSpeed(SetCommand):
    """Set start speed command configuration.

    :param store: Store flag
    :param direction: Direction
    :param speed: Speed
    :param acceleration: Acceleration
    :param en_control: Enable control
    """

    store: StoreFlag = StoreFlag.default
    direction: Direction = Direction.default
    speed: Speed = Speed.default
    acceleration: Acceleration = Acceleration.default
    en_control: EnablePin = EnablePin.default

    @property
    def _code(self) -> Code:
        return Code.SET_START_SPEED

    @property
    def _input(self) -> bytes:
        return bytes(
            [
                self.addr,
                self._code,
                Protocol.SET_START_SPEED,
                self.store,
                self.direction,
                *self.speed.bytes,
                *self.acceleration.bytes,
                self.en_control,
            ]
        )


@dataclass
class SetStartSpeedAll(SetStartSpeed, BroadcastCommand):
    """Set start speed for all motors command configuration."""


@dataclass
class SetReduction(SetCommand):
    """Set speed reduction command configuration.

    :param store: Store flag
    :param speed_reduction: Speed reduction
    """

    store: StoreFlag = StoreFlag.default
    speed_reduction: SpeedReduction = SpeedReduction.default

    @property
    def _code(self) -> Code:
        return Code.SET_REDUCTION

    @property
    def _input(self) -> bytes:
        return bytes(
            [
                self.addr,
                self._code,
                Protocol.SET_REDUCTION,
                self.store,
                self.speed_reduction,
            ]
        )


@dataclass
class SetReductionAll(SetReduction, BroadcastCommand):
    """Set speed reduction for all motors command configuration."""


@dataclass
class SetConfig(SetCommand):
    """Set configuration command configuration.

    :param store: Store flag
    :param motor_type: Motor type
    :param control_mode: Control mode
    :param comm_mode: Communication mode
    :param en_level: Enable level
    :param dir_level: Direction level
    :param microsteps: Microsteps
    :param microstep_interp: Microstep interpolation
    :param screen_off: Screen off
    :param open_loop_current: Open loop current
    :param closed_loop_current: Closed loop current
    :param max_voltage: Maximum voltage
    :param baud_rate: Baud rate
    :param can_rate: CAN rate
    :param device_id: Device ID
    :param verify_mode: Verify mode
    :param response_mode: Response mode
    :param stall_protect: Stall protection
    :param stall_speed: Stall speed
    :param stall_current: Stall current
    :param stall_time: Stall time
    :param pos_window: Position window
    """

    store: StoreFlag = StoreFlag.default
    motor_type: MotorType = MotorType.default
    control_mode: ControlMode = ControlMode.default
    communication_mode: CommunicationMode = CommunicationMode.default
    enable_level: EnableLevel = EnableLevel.default
    default_direction: DefaultDir = DefaultDir.default
    microsteps: Microstep = Microstep.default
    microstep_interp: MicrostepInterp = MicrostepInterp.default
    screen_off: ScreenOff = ScreenOff.default
    open_loop_current: OpenLoopCurrent = OpenLoopCurrent.default
    closed_loop_current: ClosedLoopCurrent = ClosedLoopCurrent.default
    max_voltage: MaxVoltage = MaxVoltage.default
    baud_rate: BaudRate = BaudRate.default
    can_rate: CanRate = CanRate.default
    device_id: Address = Address.default  # Not implemented
    checksum_mode: ChecksumMode = ChecksumMode.default
    response_mode: ResponseMode = ResponseMode.default
    stall_protect: StallProtect = StallProtect.default
    stall_speed: StallSpeed = StallSpeed.default
    stall_current: ClosedLoopCurrent = ClosedLoopCurrent.default
    stall_time: StallTime = StallTime.default
    on_target_window: OnTargetWindow = OnTargetWindow.default

    @property
    def _code(self) -> Code:
        return Code.SET_CONFIG

    @property
    def _input(self) -> bytes:
        return bytes(
            [
                self.addr,
                self._code,
                Protocol.SET_CONFIG,
                self.store,
                self.motor_type,
                self.control_mode,
                self.communication_mode,
                self.enable_level,
                self.default_direction,
                self.microsteps,
                self.microstep_interp,
                self.screen_off,
                *self.open_loop_current.bytes,
                *self.closed_loop_current.bytes,
                *self.max_voltage.bytes,
                self.baud_rate,
                self.can_rate,
                self.device_id,
                self.checksum_mode,
                self.response_mode,
                self.stall_protect,
                *self.stall_speed.bytes,
                *self.stall_current.bytes,
                *self.stall_time.bytes,
                *self.on_target_window.bytes,
            ]
        )


class SetConfigAll(SetConfig, BroadcastCommand):
    """Set configuration for all motors command configuration."""
