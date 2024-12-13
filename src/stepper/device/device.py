"""Stepper motor device class."""

import logging
import time
from typing import TypeAlias

from serial import Serial

from stepper.commands.get import (
    GetBusVoltage,
    GetConfig,
    GetEncoderValue,
    GetMotorRH,
    GetOpenLoopSetpoint,
    GetPhaseCurrent,
    GetPID,
    GetPositionError,
    GetPulseCount,
    GetRealTimePosition,
    GetRealTimeSpeed,
    GetStatus,
    GetSysStatus,
    GetTargetPosition,
    GetVersion,
)
from stepper.commands.home import (
    GetHomeStatus,
    Home,
    RetrieveHomeParam,
    SetHome,
    SetHomeParam,
    StopHome,
)
from stepper.commands.move import (
    Disable,
    Enable,
    EStop,
    Jog,
    Move,
    SyncMove,
)
from stepper.commands.set import (
    SetConfig,
    SetID,
    SetLoopMode,
    SetMicrostep,
    SetOpenLoopCurrent,
    SetPID,
    SetReduction,
    SetStartSpeed,
)
from stepper.commands.system import (
    CalibrateEncoder,
    ClearStall,
    FactoryReset,
    ZeroAllPositions,
)
from stepper.stepper_core.parameters import (
    AbsoluteFlag,
    Acceleration,
    Address,
    AutoHoming,
    BaudRate,
    BusVoltageParams,
    CanRate,
    ChecksumMode,
    ClosedLoopCurrent,
    CollisionDetectionCurrent,
    CollisionDetectionSpeed,
    CollisionDetectionTime,
    CommunicationMode,
    ConfigParams,
    ControlMode,
    DeviceParams,
    Direction,
    EnableLevel,
    EnablePin,
    EncoderParams,
    HomingDirection,
    HomingMode,
    HomingParams,
    HomingSpeed,
    HomingStatus,
    HomingTimeout,
    InputParams,
    JogParams,
    Kpid,
    LoopMode,
    MaxVoltage,
    Microstep,
    MicrostepInterp,
    MotorRHParams,
    MotorType,
    OnTargetWindow,
    OpenLoopCurrent,
    OpenLoopTargetPositionParams,
    PhaseCurrentParams,
    PIDParams,
    PositionErrorParams,
    PositionParams,
    PulseCount,
    PulseCountParams,
    RealTimePositionParams,
    RealTimeSpeedParams,
    ResponseMode,
    ScreenOff,
    Speed,
    SpeedReduction,
    StallCurrent,
    StallProtect,
    StallSpeed,
    StallTime,
    StartSpeedParams,
    StepperStatus,
    StoreFlag,
    SyncFlag,
    SystemParams,
    TargetPositionParams,
    VersionParams,
)

logger = logging.getLogger(__name__)


class Device:
    """The stepper object."""

    SerialConnection: TypeAlias = Serial
    DeviceParams: TypeAlias = DeviceParams
    InputParams: TypeAlias = InputParams | None

    def __init__(self, device_params: DeviceParams, current_params: InputParams = None):
        """Initialize the stepper object with default setups."""
        self._init_time = self.tic()
        self.tic_time = self.tic()
        self.device_params = device_params
        self.current_params = current_params or InputParams()
        self._test_connection()

        self._initial_pid: PIDParams = self.pid
        self._initial_config: ConfigParams = self.config
        self._initial_version: VersionParams = self.version
        self._initial_motor_rh: MotorRHParams = self.motor_rh
        self._initial_bus_voltage: BusVoltageParams = self.bus_voltage
        self._initial_phase_current: PhaseCurrentParams = self.phase_current
        self._initial_encoder_value: EncoderParams = self.encoder_value
        self._initial_pulse_count: PulseCountParams = self.pulse_count
        self._initial_target_position: TargetPositionParams = self.target_position
        self._initial_open_loop_setpoint: OpenLoopTargetPositionParams = self.open_loop_setpoint
        self._initial_real_time_speed: RealTimeSpeedParams = self.real_time_speed
        self._initial_real_time_position: RealTimePositionParams = self.real_time_position
        self._initial_position_error: PositionErrorParams = self.position_error
        self._initial_status: StepperStatus = self.status
        self._initial_sys_status: SystemParams = self.sys_status
        # user provided params
        self._current_jog_params: JogParams = self.current_params.jog_params
        self._current_position_params: PositionParams = self.current_params.position_params
        self._current_start_speed_params: StartSpeedParams = self.current_params.start_speed_params
        self._current_loop_mode: LoopMode = self.current_params.loop_mode
        self._current_speed_reduction: SpeedReduction = self.current_params.speed_reduction
        self._current_store_setting: StoreFlag = self.current_params.store_flag
        self._current_sync_setting: SyncFlag = self.current_params.sync_flag
        # R/W params
        self._current_config: ConfigParams = self.config
        self._current_homing_params: HomingParams = self.homing_params
        self._current_pid: PIDParams = self.pid

    def _test_connection(self) -> None:
        """Test connection to the device."""
        test = GetVersion(self.device_params)
        if not test.is_success:
            raise ConnectionError("Failed to connect to the device")
        logger.info("Connected to device successfully")

    @property
    def init_time(self) -> float:
        """Get initialization time."""
        return self._init_time

    # Read-only properties
    @property
    def version(self) -> VersionParams:
        """Get version of the device from the serial."""
        return GetVersion(self.device_params).raw_data

    @property
    def motor_rh(self) -> MotorRHParams:
        """Get motor RH parameters."""
        return GetMotorRH(self.device_params).raw_data

    @property
    def bus_voltage(self) -> BusVoltageParams:
        """Get bus voltage parameters."""
        return GetBusVoltage(self.device_params).raw_data

    @property
    def phase_current(self) -> PhaseCurrentParams:
        """Get phase current parameters."""
        return GetPhaseCurrent(self.device_params).raw_data

    @property
    def encoder_value(self) -> EncoderParams:
        """Get encoder value."""
        return GetEncoderValue(self.device_params).raw_data

    @property
    def pulse_count(self) -> PulseCountParams:
        """Get pulse count."""
        return GetPulseCount(self.device_params).raw_data

    @property
    def target_position(self) -> TargetPositionParams:
        """Get target position."""
        return GetTargetPosition(self.device_params).raw_data

    @property
    def open_loop_setpoint(self) -> OpenLoopTargetPositionParams:
        """Get open loop setpoint."""
        return GetOpenLoopSetpoint(self.device_params).raw_data

    @property
    def real_time_speed(self) -> RealTimeSpeedParams:
        """Get real time speed."""
        return GetRealTimeSpeed(self.device_params).raw_data

    @property
    def real_time_position(self) -> RealTimePositionParams:
        """Get real time position."""
        return GetRealTimePosition(self.device_params).raw_data

    @property
    def position_error(self) -> PositionErrorParams:
        """Get position error."""
        return GetPositionError(self.device_params).raw_data

    @property
    def sys_status(self) -> SystemParams:
        """Get system status."""
        return GetSysStatus(self.device_params).raw_data

    @property
    def status(self) -> StepperStatus:
        """Get status."""
        return GetStatus(self.device_params).raw_data

    @property
    def is_enabled(self) -> bool:
        """Get if the device is enabled."""
        return self.status.enabled

    @property
    def is_in_position(self) -> bool:
        """Get if the device is in position."""
        return self.status.in_position

    @property
    def is_stalled(self) -> bool:
        """Get if the device is stalled."""
        return self.status.stalled

    @property
    def is_stall_protection_active(self) -> bool:
        """Get if the stall protection is active."""
        return self.status.stall_protection_active

    # PID related methods and properties
    @property
    def pid(self) -> PIDParams:
        """Get PID parameters."""
        return GetPID(self.device_params).raw_data

    def set_pid(self, pid: PIDParams | None) -> bool:
        """Set PID parameters."""
        self._current_pid = pid or self._current_pid
        return SetPID(
            self.device_params, self._current_pid, setting=self._current_store_setting
        ).is_success

    def set_p(self, p: Kpid | int) -> bool:
        """Set PID P parameter."""
        self._current_pid.pid_p = Kpid(p)
        return self.set_pid()

    def set_i(self, i: Kpid | int) -> bool:
        """Set PID I parameter."""
        self._current_pid.pid_i = Kpid(i)
        return self.set_pid()

    def set_d(self, d: Kpid | int) -> bool:
        """Set PID D parameter."""
        self._current_pid.pid_d = Kpid(d)
        return self.set_pid()

    # Device configurations
    @property
    def config(self) -> ConfigParams:
        """Get configuration."""
        return GetConfig(self.device_params).raw_data

    def set_config(self, config: ConfigParams | None = None) -> bool:
        """Set configuration."""
        self._current_config = config or self._current_config
        SetConfig.unlock()
        return SetConfig(
            self.device_params, self._current_config, setting=self._current_store_setting
        ).is_success

    def set_stepper_type(self, stepper_type: MotorType) -> bool:
        """Set stepper type."""
        self._current_config.stepper_type = stepper_type
        return self.set_config()

    def set_control_mode(self, control_mode: ControlMode) -> bool:
        """Set control mode."""
        self._current_config.control_mode = control_mode
        return self.set_config()

    def set_communication_mode(self, communication_mode: CommunicationMode) -> bool:
        """Set communication mode."""
        self._current_config.communication_mode = communication_mode
        return self.set_config()

    def set_enable_level(self, enable_level: EnableLevel) -> bool:
        """Set enable level."""
        self._current_config.enable_level = enable_level
        return self.set_config()

    def set_default_direction(self, default_direction: Direction) -> bool:
        """Set default direction."""
        self._current_config.default_direction = default_direction
        return self.set_config()

    def set_microstep(self, microstep: Microstep | int) -> bool:
        """Set microstep."""
        microstep = Microstep(microstep)
        self._current_config.microsteps = microstep
        return SetMicrostep(
            self.device_params, microstep, setting=self._current_store_setting
        ).is_success

    def set_microstep_interp(self, microstep_interp: MicrostepInterp) -> bool:
        """Set microstep interpolation."""
        self._current_config.microstep_interp = microstep_interp
        return self.set_config()

    def set_screen_off(self, screen_off: ScreenOff) -> bool:
        """Set screen off."""
        self._current_config.screen_off = screen_off
        return self.set_config()

    def set_open_loop_current(self, open_loop_current: OpenLoopCurrent | int) -> bool:
        """Set open loop current."""
        open_loop_current = OpenLoopCurrent(open_loop_current)
        self._current_config.open_loop_current = open_loop_current
        return SetOpenLoopCurrent(
            self.device_params, open_loop_current, setting=self._current_store_setting
        ).is_success

    def set_max_closed_loop_current(self, max_closed_loop_current: ClosedLoopCurrent | int) -> bool:
        """Set max closed loop current."""
        max_closed_loop_current = ClosedLoopCurrent(max_closed_loop_current)
        self._current_config.max_closed_loop_current = max_closed_loop_current
        return self.set_config()

    def set_max_voltage(self, max_voltage: MaxVoltage | int) -> bool:
        """Set max voltage."""
        max_voltage = MaxVoltage(max_voltage)
        self._current_config.max_voltage = max_voltage
        return self.set_config()

    def set_baud_rate(self, baud_rate: BaudRate | int) -> bool:
        """Set baud rate."""
        if isinstance(baud_rate, int):
            baud_rate = BaudRate.from_value(baud_rate)
        self._current_config.baud_rate = baud_rate
        return self.set_config()

    def set_canrate(self, can_rate: CanRate | int) -> bool:
        """Set CAN rate."""
        if isinstance(can_rate, int):
            can_rate = CanRate.from_value(can_rate)
        self._current_config.can_rate = can_rate
        return self.set_config()

    def set_id(self, id: Address) -> bool:
        """Set ID."""
        success = SetID(self.device_params, id, setting=self._current_store_setting).is_success
        if success:
            self._current_config.address = id
            self.device_params.address = id
        self._test_connection()
        return success

    def set_checksum_mode(self, checksum_mode: ChecksumMode) -> bool:
        """Set checksum mode."""
        self._current_config.checksum_mode = checksum_mode
        return self.set_config()

    def set_response_mode(self, response_mode: ResponseMode) -> bool:
        """Set response mode."""
        self._current_config.response_mode = response_mode
        return self.set_config()

    def set_stall_protect(self, stall_protect: StallProtect) -> bool:
        """Set stall protect."""
        self._current_config.stall_protect = stall_protect
        return self.set_config()

    def set_stall_speed(self, stall_speed: StallSpeed | int) -> bool:
        """Set stall speed."""
        self._current_config.stall_speed = StallSpeed(stall_speed)
        return self.set_config()

    def set_stall_current(self, stall_current: StallCurrent | int) -> bool:
        """Set stall current."""
        self._current_config.stall_current = StallCurrent(stall_current)
        return self.set_config()

    def set_stall_time(self, stall_time: StallTime | int) -> bool:
        """Set stall time."""
        self._current_config.stall_time = StallTime(stall_time)
        return self.set_config()

    def set_on_target_window(self, on_target_window: OnTargetWindow | int | float) -> bool:
        """Set on target window."""
        if isinstance(on_target_window, float):
            on_target_window = int(on_target_window * 10)
        if isinstance(on_target_window, int):
            on_target_window = OnTargetWindow(on_target_window)
        self._current_config.on_target_window = on_target_window
        return self.set_config()

    # Start speed related methods and properties
    def set_start_speed_params(self, start_speed_params: StartSpeedParams | None = None) -> bool:
        """Set start speed configuration."""
        self._current_start_speed_params = start_speed_params or self._current_start_speed_params
        return SetStartSpeed(
            self.device_params,
            self._current_start_speed_params,
            setting=self._current_store_setting,
        ).is_success

    def set_start_direction(self, start_direction: Direction) -> bool:
        """Set start direction."""
        self._current_start_speed_params.direction = start_direction
        return self.set_start_speed_params()

    def set_start_speed(self, start_speed: Speed | int) -> bool:
        """Set start speed."""
        if isinstance(start_speed, int):
            if start_speed < 0:
                self._current_start_speed_params.direction = Direction.CCW
                start_speed = Speed(-start_speed)
            else:
                self._current_start_speed_params.direction = Direction.CW
                start_speed = Speed(start_speed)
        self._current_start_speed_params.speed = start_speed
        return self.set_start_speed_params()

    def set_start_acceleration(self, start_acceleration: Acceleration | int) -> bool:
        """Set start acceleration."""
        self._current_start_speed_params.acceleration = Acceleration(start_acceleration)
        return self.set_start_speed_params()

    def set_start_en_control(self, start_en_control: EnablePin) -> bool:
        """Set start en control."""
        self._current_start_speed_params.en_control = start_en_control
        return self.set_start_speed_params()

    # System configuration related methods
    def set_loop_mode(self, loop_mode: LoopMode) -> bool:
        """Set loop mode."""
        self._current_loop_mode = loop_mode
        return SetLoopMode(
            self.device_params, self._current_loop_mode, setting=self._current_store_setting
        ).is_success

    def set_speed_reduction(self, speed_reduction: SpeedReduction) -> bool:
        """Set speed reduction."""
        self._current_speed_reduction = speed_reduction
        return SetReduction(
            self.device_params, self._current_speed_reduction, setting=self._current_store_setting
        ).is_success

    def sys_calibrate_encoder(self) -> bool:
        """System calibrate encoder."""
        CalibrateEncoder.unlock()
        return CalibrateEncoder(self.device_params).is_success

    def sys_factory_reset(self) -> bool:
        """System factory reset."""
        FactoryReset.unlock()
        return FactoryReset(self.device_params).is_success

    def sys_clear_stall(self) -> bool:
        """System clear stall."""
        ClearStall.unlock()
        return ClearStall(self.device_params).is_success

    def sys_zero_all_positions(self) -> bool:
        """System zero all positions."""
        ZeroAllPositions.unlock()
        return ZeroAllPositions(self.device_params).is_success

    # Homing related methods and properties
    @property
    def homing_params(self) -> HomingParams:
        """Retrieve homing parameters."""
        return RetrieveHomeParam(self.device_params).raw_data

    @property
    def homing_status(self) -> HomingStatus:
        """Get homing status."""
        return GetHomeStatus(self.device_params).raw_data

    @property
    def encoder_ready(self) -> bool:
        """Get if the encoder is ready."""
        return self.homing_status.encoder_ready

    @property
    def encoder_calibrated(self) -> bool:
        """Get if the encoder is calibrated."""
        return self.homing_status.encoder_calibrated

    @property
    def is_homing(self) -> bool:
        """Get if the device is homing."""
        return self.homing_status.is_homing

    @property
    def is_homing_failed(self) -> bool:
        """Get if the homing failed."""
        return self.homing_status.homing_failed

    def set_homing_params(self, homing_params: HomingParams | None = None) -> bool:
        """Set homing parameters."""
        self._current_homing_params = homing_params or self._current_homing_params
        SetHomeParam.unlock()
        return SetHomeParam(
            self.device_params, self._current_homing_params, setting=self._current_store_setting
        ).is_success

    def set_homing_mode(self, homing_mode: HomingMode) -> bool:
        """Set homing mode."""
        self._current_homing_params.homing_mode = homing_mode
        return self.set_homing_params()

    def set_homing_direction(self, homing_direction: HomingDirection) -> bool:
        """Set homing direction."""
        self._current_homing_params.homing_direction = homing_direction
        return self.set_homing_params()

    def set_homing_speed(self, homing_speed: HomingSpeed | int) -> bool:
        """Set homing speed."""
        self._current_homing_params.homing_speed = HomingSpeed(homing_speed)
        return self.set_homing_params()

    def set_homing_timeout(self, homing_timeout: HomingTimeout | int) -> bool:
        """Set homing timeout."""
        self._current_homing_params.homing_timeout = HomingTimeout(homing_timeout)
        return self.set_homing_params()

    def set_collision_detection_speed(
        self, collision_detection_speed: CollisionDetectionSpeed | int
    ) -> bool:
        """Set collision detection speed."""
        self._current_homing_params.collision_detection_speed = CollisionDetectionSpeed(
            collision_detection_speed
        )
        return self.set_homing_params()

    def set_collision_detection_current(
        self, collision_detection_current: CollisionDetectionCurrent | int
    ) -> bool:
        """Set collision detection current."""
        self._current_homing_params.collision_detection_current = CollisionDetectionCurrent(
            collision_detection_current
        )
        return self.set_homing_params()

    def set_collision_detection_time(
        self, collision_detection_time: CollisionDetectionTime | int
    ) -> bool:
        """Set collision detection time."""
        self._current_homing_params.collision_detection_time = CollisionDetectionTime(
            collision_detection_time
        )
        return self.set_homing_params()

    def set_auto_home(self, auto_home: AutoHoming) -> bool:
        """Set auto home."""
        self._current_homing_params.auto_home = auto_home
        return self.set_homing_params()

    def home(self) -> bool:
        """Return to the home position."""
        if not self.is_enabled:
            self.enable()
        return Home(self.device_params).is_success

    def set_home(self) -> bool:
        """Set the current position as home."""
        return SetHome(self.device_params, setting=self._current_store_setting).is_success

    def stop_home(self) -> bool:
        """Stop homing."""
        return StopHome(self.device_params).is_success

    # move related methods and properties
    def enable(self) -> bool:
        """Enable the stepper."""
        if not self.is_enabled:
            return Enable(self.device_params, setting=self._current_sync_setting).is_success
        return self.is_enabled

    def disable(self) -> bool:
        """Disable the stepper."""
        if self.is_enabled:
            return Disable(self.device_params, setting=self._current_sync_setting).is_success
        return not self.is_enabled

    def estop(self) -> bool:
        """Emergency stop."""
        return EStop(self.device_params, setting=self._current_sync_setting).is_success

    def jog(self, jog_params: JogParams | int | Speed | None = None) -> bool:
        """Jog in a direction."""
        if isinstance(jog_params, int):
            if jog_params < 0:
                self._current_jog_params.direction = Direction.CCW
                self._current_jog_params.speed = Speed(-jog_params)
            else:
                self._current_jog_params.direction = Direction.CW
                self._current_jog_params.speed = Speed(jog_params)
        elif isinstance(jog_params, Speed):
            self._current_jog_params.speed = jog_params
        elif isinstance(jog_params, JogParams):
            self._current_jog_params = jog_params
        elif jog_params is None:
            ...
        if not self.is_enabled:
            self.enable()
        return Jog(
            self.device_params, self._current_jog_params, setting=self._current_sync_setting
        ).is_success

    def jog_cw(self) -> bool:
        """Jog clockwise."""
        self._current_jog_params.direction = Direction.CW
        return self.jog()

    def jog_ccw(self) -> bool:
        """Jog counterclockwise."""
        self._current_jog_params.direction = Direction.CCW
        return self.jog()

    def jog_at_speed(self, speed: Speed | int) -> bool:
        """Jog at a speed."""
        return self.jog(jog_params=speed)

    def set_jog_speed(self, speed: Speed | int) -> bool:
        """Set jog speed."""
        self._current_jog_params.speed = Speed(speed)
        return self._current_jog_params.speed == speed

    def set_jog_direction(self, direction: Direction) -> bool:
        """Set jog direction."""
        self._current_jog_params.direction = direction
        return self._current_jog_params.direction == direction

    def set_jog_acceleration(self, acceleration: Acceleration | int) -> bool:
        """Set jog acceleration."""
        self._current_jog_params.acceleration = Acceleration(acceleration)
        return self._current_jog_params.acceleration == acceleration

    def stop(self) -> bool:
        """Stop jog."""
        return self.jog(0)

    def move(self, position_params: PositionParams | int | PulseCount | None = None) -> bool:
        """Move to a position."""
        if isinstance(position_params, int):
            if position_params < 0:
                self._current_position_params.direction = Direction.CCW
                self._current_position_params.pulse_count = PulseCount(-position_params)
            else:
                self._current_position_params.direction = Direction.CW
                self._current_position_params.pulse_count = PulseCount(position_params)
        elif isinstance(position_params, PulseCount):
            self._current_position_params.pulse_count = position_params
        elif isinstance(position_params, PositionParams):
            self._current_position_params = position_params
        elif position_params is None:
            ...
        if not self.is_enabled:
            self.enable()
        return Move(
            self.device_params, self._current_position_params, setting=self._current_sync_setting
        ).is_success

    def move_to(self, position: PulseCount | int) -> bool:
        """Move to a position."""
        self._current_position_params.absolute = AbsoluteFlag.ABSOLUTE
        if isinstance(position, int):
            if position < 0:
                self._current_position_params.direction = Direction.CCW
                self._current_position_params.pulse_count = PulseCount(-position)
            else:
                self._current_position_params.direction = Direction.CW
                self._current_position_params.pulse_count = PulseCount(position)
        return self.move()

    def move_cw(self, distance: PulseCount | int) -> bool:
        """Move clockwise."""
        self._current_position_params.direction = Direction.CW
        self._current_position_params.absolute = AbsoluteFlag.RELATIVE
        self._current_position_params.pulse_count = PulseCount(distance)
        return self.move()

    def move_ccw(self, distance: PulseCount | int) -> bool:
        """Move counterclockwise."""
        self._current_position_params.direction = Direction.CCW
        self._current_position_params.absolute = AbsoluteFlag.RELATIVE
        self._current_position_params.pulse_count = PulseCount(distance)
        return self.move()

    def set_move_speed(self, speed: Speed | int) -> bool:
        """Set move speed."""
        self._current_position_params.speed = Speed(speed)
        return self._current_position_params.speed == speed

    def set_move_direction(self, direction: Direction) -> bool:
        """Set move direction."""
        self._current_position_params.direction = direction
        return self._current_position_params.direction == direction

    def set_move_acceleration(self, acceleration: Acceleration | int) -> bool:
        """Set move acceleration."""
        self._current_position_params.acceleration = Acceleration(acceleration)
        return self._current_position_params.acceleration == acceleration

    def set_speed(self, speed: Speed | int) -> bool:
        """Set speed."""
        self._current_position_params.speed = Speed(speed)
        self._current_jog_params.speed = Speed(speed)
        return self._current_position_params.speed == speed

    def set_direction(self, direction: Direction) -> bool:
        """Set direction."""
        self._current_position_params.direction = direction
        self._current_jog_params.direction = direction
        return self._current_position_params.direction == direction

    def set_acceleration(self, acceleration: Acceleration | int) -> bool:
        """Set acceleration."""
        self._current_position_params.acceleration = Acceleration(acceleration)
        self._current_jog_params.acceleration = Acceleration(acceleration)
        return self._current_position_params.acceleration == acceleration

    def sync_move(self) -> bool:
        """Sync move to a position."""
        if not self.is_enabled:
            self.enable()
        return SyncMove(self.device_params, setting=self._current_sync_setting).is_success

    def wait(self, delay: float = 1.0) -> bool:
        """Delay."""
        time.sleep(delay)
        logger.info(f"Delayed for {delay} seconds.")
        return True

    @property
    def jog_direction(self) -> Direction:
        """Get jog direction."""
        return self._current_jog_params.direction

    @property
    def jog_speed(self) -> Speed:
        """Get jog speed."""
        return self._current_jog_params.speed

    @property
    def jog_acceleration(self) -> Acceleration:
        """Get jog acceleration."""
        return self._current_jog_params.acceleration

    @property
    def move_speed(self) -> Speed:
        """Get move speed."""
        return self._current_position_params.speed

    @property
    def move_acceleration(self) -> Acceleration:
        """Get move acceleration."""
        return self._current_position_params.acceleration

    @property
    def move_direction(self) -> Direction:
        """Get move direction."""
        return self._current_position_params.direction

    @property
    def move_pulse_count(self) -> PulseCount:
        """Get move pulse count."""
        return self._current_position_params.pulse_count

    @property
    def move_mode(self) -> AbsoluteFlag:
        """Get move mode."""
        return self._current_position_params.absolute

    @property
    def is_sync(self) -> bool:
        """Get if the device is sync."""
        return self._current_sync_setting == SyncFlag.SYNC

    def enable_sync(self) -> bool:
        """Enable sync."""
        self._current_sync_setting = SyncFlag.SYNC
        return self.is_sync

    def disable_sync(self) -> bool:
        """Disable sync."""
        self._current_sync_setting = SyncFlag.NO_SYNC
        return not self.is_sync

    def enable_store(self) -> bool:
        """Enable store."""
        self._current_store_setting = StoreFlag.PERMANENT
        return self.is_store

    def disable_store(self) -> bool:
        """Disable store."""
        self._current_store_setting = StoreFlag.TEMPORARY
        return not self.is_store

    @property
    def is_store(self) -> bool:
        """Get if the device is store."""
        return self._current_store_setting == StoreFlag.PERMANENT

    def debug(self) -> list[str]:
        """Debug the device."""
        potential_bug = []
        if self.is_homing:
            logger.info("The device is homing.")
            potential_bug.append("is homing")
        if self.is_homing_failed:
            logger.info("The device is homing failed.")
            potential_bug.append("homing failed")
        if not self.encoder_calibrated:
            logger.info("The device is not calibrated.")
            potential_bug.append("not calibrated")
        if not self.encoder_ready:
            logger.info("The device is not ready.")
            potential_bug.append("not ready")
        if not self.is_enabled:
            logger.info("The device is not enabled.")
            potential_bug.append("not enabled")
        if not self.is_in_position:
            logger.info("The device is not in position.")
            potential_bug.append("not in position")
        if self.is_stalled:
            logger.info("The device is stalled.")
            potential_bug.append("stalled")
        if self.is_stall_protection_active:
            logger.info("The device is stall protection active.")
            potential_bug.append("stall protection active")
        return potential_bug

    def resolve_bug(self) -> bool:
        """Resolve the bug. May take 30 seconds."""
        if self.is_stalled:
            logger.info("Clearing stall.")
            self.sys_clear_stall()
            self.wait(3)
            return self.is_stalled

        if self.is_homing:
            logger.info("Stopping homing.")
            self.stop_home()
            self.wait(3)
            return self.is_homing

        if self.is_homing_failed:
            logger.info("Homing again.")
            self.home()
            self.wait(30)
            return self.is_homing_failed

        if not self.encoder_calibrated:
            logger.info("Calibrating encoder.")
            self.sys_calibrate_encoder()
            self.wait(30)
            return self.encoder_calibrated

        if not self.encoder_ready:
            logger.info("Calibrating encoder again.")
            self.sys_calibrate_encoder()
            self.wait(30)
            return self.encoder_ready

        if not self.is_enabled:
            logger.info("Enabling the device.")
            self.enable()
            self.wait(3)
            return self.is_enabled

        if not self.is_in_position:
            logger.info("Moving to the home position.")
            self.move_to(0)
            self.wait(30)
            return self.is_in_position

    @property
    def state_dict(self) -> dict:
        """Get the device state dictionary."""
        state_dict = {}
        state_dict.update(self.config.data_dict)
        state_dict.update(self.sys_status.data_dict)
        state_dict.update(self.pid.data_dict)
        return state_dict

    @property
    def params_dict(self) -> dict:
        """Get the device params dictionary."""
        params_dict = {}
        params_dict.update(self.device_params.__dict__)
        params_dict.update(self._current_jog_params.data_dict)
        params_dict.update(self._current_position_params.data_dict)
        params_dict.update({"speed_reduction": self._current_speed_reduction.name})
        params_dict.update(self._current_start_speed_params.data_dict)
        params_dict.update({"sync_flag": self._current_sync_setting.name})
        params_dict.update({"store_flag": self._current_store_setting.name})
        params_dict.update(self.config.data_dict)
        params_dict.update(self.pid.data_dict)
        return params_dict

    def tic(self) -> float:
        """Start timing.

        :return: Current time in seconds with high precision
        """
        self.tic_time = time.perf_counter()
        return self.tic_time

    def toc(self) -> float:
        """Get elapsed time since last tic.

        :return: Elapsed time in seconds with high precision
        """
        return time.perf_counter() - self.tic_time

    def jog_time(self, time: float, jog_params: JogParams | int | Speed | None = None) -> bool:
        """Jog time."""
        self.tic()
        self.jog(jog_params)
        self.wait(time * 0.95)
        while self.toc() < time:
            ...
        self.stop()
        return True

    def parse_cmd(self, command: str) -> str | None:
        """Parse and execute stepper command string.

        :param command: Command string using 3-character codes
        :return: Formatted string with timestamp, command and result if successful, None if failed

        Movement Commands:
        HOM - Home the device
        SHM - Set current position as home
        MOV {pos} - Move to absolute position
        MRL {pos} - Move relative distance
        JOG {speed} - Jog at speed (continuous movement)
        JCW {speed} - Jog clockwise
        JCC {speed} - Jog counter-clockwise
        STP - Stop all movement

        Mode Settings:
        ABS - Set absolute positioning mode
        REL - Set relative positioning mode
        SYN - Enable synchronized movement
        NSY - Disable synchronized movement
        STO - Enable parameter storage
        TMP - Disable parameter storage (temporary)

        Motor Control:
        ENA - Enable stepper
        DIS - Disable stepper
        EST - Emergency stop
        WAI {seconds} - Wait specified seconds
        WIP - Wait for in-position
        CLR - Clear stall condition

        Parameter Settings:
        ACC {value} - Set acceleration
        SPD {value} - Set max speed
        CUR {value} - Set current
        DIR {0/1} - Set direction (0:CW, 1:CCW)
        PID {p} {i} {d} - Set PID parameters

        Status Commands:
        POS - Get current position
        STA - Get status
        DBG - Get debug info
        VER - Get firmware version
        VOL - Get bus voltage
        TMP - Get temperature
        ERR - Get position error
        """
        try:
            parts = command.strip().upper().split()
            timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
            match parts:
                case [cmd]:
                    match cmd:
                        case "HOM":
                            return f"{timestamp}: {cmd} = {self.home()}"
                        case "SHM":
                            return f"{timestamp}: {cmd} = {self.set_home()}"
                        case "STP":
                            return f"{timestamp}: {cmd} = {self.stop()}"
                        case "ABS":
                            self._current_position_params.absolute = AbsoluteFlag.ABSOLUTE
                            return f"{timestamp}: {cmd}"
                        case "REL":
                            self._current_position_params.absolute = AbsoluteFlag.RELATIVE
                            return f"{timestamp}: {cmd}"
                        case "SYN":
                            return f"{timestamp}: {cmd} = {self.enable_sync()}"
                        case "NSY":
                            return f"{timestamp}: {cmd} = {self.disable_sync()}"
                        case "STO":
                            return f"{timestamp}: {cmd} = {self.enable_store()}"
                        case "TMP":
                            return f"{timestamp}: {cmd} = {self.disable_store()}"
                        case "ENA":
                            return f"{timestamp}: {cmd} = {self.enable()}"
                        case "DIS":
                            return f"{timestamp}: {cmd} = {self.disable()}"
                        case "EST":
                            return f"{timestamp}: {cmd} = {self.estop()}"
                        case "WIP":
                            start_time = time.perf_counter()
                            while not self.is_in_position:
                                if time.perf_counter() - start_time > 30:
                                    logger.warning("Waited too long for in-position")
                                    return f"{timestamp}: {cmd} = False"
                                self.wait(0.1)
                            return f"{timestamp}: {cmd} = True"
                        case "CLR":
                            return f"{timestamp}: {cmd} = {self.sys_clear_stall()}"
                        case "POS":
                            pos = self.real_time_position.position
                            logger.info(f"Position: {pos}")
                            return f"{timestamp}: {cmd} = {pos}"
                        case "STA":
                            status = []
                            if self.is_enabled:
                                status.append("ENABLED")
                            if self.is_in_position:
                                status.append("IN_POS")
                            if self.is_stalled:
                                status.append("STALLED")
                            if self.is_homing:
                                status.append("HOMING")
                            if self.is_sync:
                                status.append("SYNC")
                            if self.is_store:
                                status.append("STORE")
                            status_str = " ".join(status)
                            logger.info(f"Status: {status_str}")
                            return f"{timestamp}: {cmd} = {status_str}"
                        case "VER":
                            ver = self.version
                            logger.info(f"Firmware version: {ver}")
                            return f"{timestamp}: {cmd} = {ver}"
                        case "VOL":
                            vol = self.bus_voltage
                            logger.info(f"Bus voltage: {vol}")
                            return f"{timestamp}: {cmd} = {vol}"
                        case "ERR":
                            err = self.position_error
                            logger.info(f"Position error: {err}")
                            return f"{timestamp}: {cmd} = {err}"
                        case _:
                            logger.warning(f"Unknown command: {cmd}")
                            return None

                case [cmd, param, *rest]:
                    match cmd:
                        case "MOV":
                            return f"{timestamp}: {cmd} {param} = {self.move_to(int(param))}"
                        case "MRL":
                            pos = int(param)
                            result = self.move_cw(pos) if pos >= 0 else self.move_ccw(abs(pos))
                            return f"{timestamp}: {cmd} {param} = {result}"
                        case "JOG":
                            return f"{timestamp}: {cmd} {param} = {self.jog(int(param))}"
                        case "JCW":
                            self._current_jog_params.direction = Direction.CW
                            return f"{timestamp}: {cmd} {param} = {self.jog(int(param))}"
                        case "JCC":
                            self._current_jog_params.direction = Direction.CCW
                            return f"{timestamp}: {cmd} {param} = {self.jog(int(param))}"
                        case "WAI":
                            return f"{timestamp}: {cmd} {param} = {self.wait(float(param))}"
                        case "ACC":
                            return (
                                f"{timestamp}: {cmd} {param} = {self.set_acceleration(int(param))}"
                            )
                        case "SPD":
                            return f"{timestamp}: {cmd} {param} = {self.set_speed(int(param))}"
                        case "CUR":
                            return f"{timestamp}: {cmd} {param} = {self.set_open_loop_current(int(param))}"  # noqa
                        case "DIR":
                            if param not in ["0", "1"]:
                                logger.error("DIR requires 0 (CW) or 1 (CCW)")
                                return None
                            result = self.set_direction(
                                Direction.CCW if param == "1" else Direction.CW
                            )
                            return f"{timestamp}: {cmd} {param} = {result}"
                        case "PID":
                            if len(rest) != 2:
                                logger.error("PID requires P, I, D parameters")
                                return None
                            success = all(
                                [
                                    self.set_p(int(param)),
                                    self.set_i(int(rest[0])),
                                    self.set_d(int(rest[1])),
                                ]
                            )
                            return f"{timestamp}: {cmd} {param} {rest[0]} {rest[1]} = {success}"
                        case _:
                            logger.warning(f"Unknown command: {cmd}")
                            return None

                case _:
                    logger.warning(f"Invalid command format: {command}")
                    return None

        except (ValueError, IndexError) as e:
            logger.error(f"Failed to parse command '{command}': {str(e)}")
            return None
