"""Stepper motor device class."""

import logging
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
    def sys_status(self) -> SystemParams:
        """Get system status."""
        return GetSysStatus(self.device_params).raw_data

    @property
    def is_stall_protection_active(self) -> bool:
        """Get if the stall protection is active."""
        return self.status.stall_protection_active

    # PID related methods and properties
    @property
    def pid(self) -> PIDParams:
        """Get PID parameters."""
        return GetPID(self.device_params).raw_data

    def set_pid(self, pid: PIDParams) -> bool:
        """Set PID parameters."""
        self._current_pid = pid
        SetPID.unlock()
        return SetPID(self.device_params, self._current_pid).is_success

    def set_p(self, p: Kpid | int) -> bool:
        """Set PID P parameter."""
        self._current_pid.pid_p = Kpid(p)
        SetPID.unlock()
        return SetPID(self.device_params, self._current_pid).is_success

    def set_i(self, i: Kpid | int) -> bool:
        """Set PID I parameter."""
        self._current_pid.pid_i = Kpid(i)
        SetPID.unlock()
        return SetPID(self.device_params, self._current_pid).is_success

    def set_d(self, d: Kpid | int) -> bool:
        """Set PID D parameter."""
        self._current_pid.pid_d = Kpid(d)
        SetPID.unlock()
        return SetPID(self.device_params, self._current_pid).is_success

    # Device configurations
    @property
    def config(self) -> ConfigParams:
        """Get configuration."""
        return GetConfig(self.device_params).raw_data

    def set_config(self, config: ConfigParams) -> bool:
        """Set configuration."""
        self._current_config = config
        SetConfig.unlock()
        return SetConfig(self.device_params, self._current_config).is_success

    def set_stepper_type(self, stepper_type: MotorType) -> bool:
        """Set stepper type."""
        self._current_config.stepper_type = stepper_type
        SetConfig.unlock()
        return SetConfig(self.device_params, self._current_config).is_success

    def set_control_mode(self, control_mode: ControlMode) -> bool:
        """Set control mode."""
        self._current_config.control_mode = control_mode
        SetConfig.unlock()
        return SetConfig(self.device_params, self._current_config).is_success

    def set_communication_mode(self, communication_mode: CommunicationMode) -> bool:
        """Set communication mode."""
        self._current_config.communication_mode = communication_mode
        SetConfig.unlock()
        return SetConfig(self.device_params, self._current_config).is_success

    def set_enable_level(self, enable_level: EnableLevel) -> bool:
        """Set enable level."""
        self._current_config.enable_level = enable_level
        SetConfig.unlock()
        return SetConfig(self.device_params, self._current_config).is_success

    def set_default_direction(self, default_direction: Direction) -> bool:
        """Set default direction."""
        self._current_config.default_direction = default_direction
        SetConfig.unlock()
        return SetConfig(self.device_params, self._current_config).is_success

    def set_microstep(self, microstep: Microstep | int) -> bool:
        """Set microstep."""
        microstep = Microstep(microstep)
        self._current_config.microsteps = microstep
        SetMicrostep.unlock()
        return SetMicrostep(self.device_params, self._current_config).is_success

    def set_microstep_interp(self, microstep_interp: MicrostepInterp) -> bool:
        """Set microstep interpolation."""
        self._current_config.microstep_interp = microstep_interp
        SetConfig.unlock()
        return SetConfig(self.device_params, self._current_config).is_success

    def set_screen_off(self, screen_off: ScreenOff) -> bool:
        """Set screen off."""
        self._current_config.screen_off = screen_off
        SetConfig.unlock()
        return SetConfig(self.device_params, self._current_config).is_success

    def set_open_loop_current(self, open_loop_current: OpenLoopCurrent | int) -> bool:
        """Set open loop current."""
        open_loop_current = OpenLoopCurrent(open_loop_current)
        self._current_config.open_loop_current = open_loop_current
        SetOpenLoopCurrent.unlock()
        return SetOpenLoopCurrent(self.device_params, self._current_config).is_success

    def set_max_closed_loop_current(self, max_closed_loop_current: ClosedLoopCurrent | int) -> bool:
        """Set max closed loop current."""
        max_closed_loop_current = ClosedLoopCurrent(max_closed_loop_current)
        self._current_config.max_closed_loop_current = max_closed_loop_current
        SetConfig.unlock()
        return SetConfig(self.device_params, self._current_config).is_success

    def set_max_voltage(self, max_voltage: MaxVoltage | int) -> bool:
        """Set max voltage."""
        max_voltage = MaxVoltage(max_voltage)
        self._current_config.max_voltage = max_voltage
        SetConfig.unlock()
        return SetConfig(self.device_params, self._current_config).is_success

    def set_baud_rate(self, baud_rate: BaudRate | int) -> bool:
        """Set baud rate."""
        if isinstance(baud_rate, int):
            baud_rate = BaudRate.from_value(baud_rate)
        self._current_config.baud_rate = baud_rate
        SetConfig.unlock()
        return SetConfig(self.device_params, self._current_config).is_success

    def set_canrate(self, can_rate: CanRate | int) -> bool:
        """Set CAN rate."""
        if isinstance(can_rate, int):
            can_rate = CanRate.from_value(can_rate)
        self._current_config.can_rate = can_rate
        SetConfig.unlock()
        return SetConfig(self.device_params, self._current_config).is_success

    def set_id(self, id: Address) -> bool:
        """Set ID."""
        self._current_config.address = id
        SetID.unlock()
        return SetID(self.device_params, self._current_config).is_success

    def set_checksum_mode(self, checksum_mode: ChecksumMode) -> bool:
        """Set checksum mode."""
        self._current_config.checksum_mode = checksum_mode
        SetConfig.unlock()
        return SetConfig(self.device_params, self._current_config).is_success

    def set_response_mode(self, response_mode: ResponseMode) -> bool:
        """Set response mode."""
        self._current_config.response_mode = response_mode
        SetConfig.unlock()
        return SetConfig(self.device_params, self._current_config).is_success

    def set_stall_protect(self, stall_protect: StallProtect) -> bool:
        """Set stall protect."""
        self._current_config.stall_protect = stall_protect
        SetConfig.unlock()
        return SetConfig(self.device_params, self._current_config).is_success

    def set_stall_speed(self, stall_speed: StallSpeed | int) -> bool:
        """Set stall speed."""
        self._current_config.stall_speed = StallSpeed(stall_speed)
        SetConfig.unlock()
        return SetConfig(self.device_params, self._current_config).is_success

    def set_stall_current(self, stall_current: StallCurrent | int) -> bool:
        """Set stall current."""
        self._current_config.stall_current = StallCurrent(stall_current)
        SetConfig.unlock()
        return SetConfig(self.device_params, self._current_config).is_success

    def set_stall_time(self, stall_time: StallTime | int) -> bool:
        """Set stall time."""
        self._current_config.stall_time = StallTime(stall_time)
        SetConfig.unlock()
        return SetConfig(self.device_params, self._current_config).is_success

    def set_on_target_window(self, on_target_window: OnTargetWindow | int | float) -> bool:
        """Set on target window."""
        if isinstance(on_target_window, float):
            on_target_window = int(on_target_window * 10)
        elif isinstance(on_target_window, int):
            on_target_window = OnTargetWindow(on_target_window)
        self._current_config.on_target_window = on_target_window
        SetConfig.unlock()
        return SetConfig(self.device_params, self._current_config).is_success

    # Start speed related methods and properties
    def set_start_speed_params(self, start_speed_params: StartSpeedParams) -> bool:
        """Set start speed configuration."""
        self._current_start_speed_params = start_speed_params
        SetStartSpeed.unlock()
        return SetStartSpeed(self.device_params, self._current_start_speed_params).is_success

    def set_start_direction(self, start_direction: Direction) -> bool:
        """Set start direction."""
        self._current_start_speed_params.direction = start_direction
        SetStartSpeed.unlock()
        return SetStartSpeed(self.device_params, self._current_start_speed_params).is_success

    def set_start_speed(self, start_speed: Speed | int) -> bool:
        """Set start speed."""
        if isinstance(start_speed, int):
            if start_speed < 0:
                logger.info(f"Setting start speed to {start_speed} (CCW)")
                self._current_start_speed_params.direction = Direction.CCW
                start_speed = Speed(-start_speed)
            else:
                logger.info(f"Setting start speed to {start_speed} (CW)")
                self._current_start_speed_params.direction = Direction.CW
                start_speed = Speed(start_speed)
        self._current_start_speed_params.speed = start_speed
        SetStartSpeed.unlock()
        return SetStartSpeed(self.device_params, self._current_start_speed_params).is_success

    def set_start_acceleration(self, start_acceleration: Acceleration | int) -> bool:
        """Set start acceleration."""
        self._current_start_speed_params.acceleration = Acceleration(start_acceleration)
        SetStartSpeed.unlock()
        return SetStartSpeed(self.device_params, self._current_start_speed_params).is_success

    def set_start_en_control(self, start_en_control: EnablePin) -> bool:
        """Set start en control."""
        self._current_start_speed_params.en_control = start_en_control
        SetStartSpeed.unlock()
        return SetStartSpeed(self.device_params, self._current_start_speed_params).is_success

    # System configuration related methods
    def set_loop_mode(self, loop_mode: LoopMode) -> bool:
        """Set loop mode."""
        self._current_loop_mode = loop_mode
        SetLoopMode.unlock()
        return SetLoopMode(self.device_params, self._current_loop_mode).is_success

    def set_speed_reduction(self, speed_reduction: SpeedReduction) -> bool:
        """Set speed reduction."""
        self._current_speed_reduction = speed_reduction
        SetReduction.unlock()
        return SetReduction(self.device_params, self._current_speed_reduction).is_success

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

    def set_homing_params(self, homing_params: HomingParams) -> bool:
        """Set homing parameters."""
        self._current_homing_params = homing_params
        SetHomeParam.unlock()
        return SetHomeParam(self.device_params, self._current_homing_params).is_success

    def set_homing_mode(self, homing_mode: HomingMode) -> bool:
        """Set homing mode."""
        self._current_homing_params.homing_mode = homing_mode
        SetHomeParam.unlock()
        return SetHomeParam(self.device_params, self._current_homing_params).is_success

    def set_homing_direction(self, homing_direction: HomingDirection) -> bool:
        """Set homing direction."""
        self._current_homing_params.homing_direction = homing_direction
        SetHomeParam.unlock()
        return SetHomeParam(self.device_params, self._current_homing_params).is_success

    def set_homing_speed(self, homing_speed: HomingSpeed | int) -> bool:
        """Set homing speed."""
        self._current_homing_params.homing_speed = HomingSpeed(homing_speed)
        SetHomeParam.unlock()
        return SetHomeParam(self.device_params, self._current_homing_params).is_success

    def set_homing_timeout(self, homing_timeout: HomingTimeout | int) -> bool:
        """Set homing timeout."""
        self._current_homing_params.homing_timeout = HomingTimeout(homing_timeout)
        SetHomeParam.unlock()
        return SetHomeParam(self.device_params, self._current_homing_params).is_success

    def set_collision_detection_speed(
        self, collision_detection_speed: CollisionDetectionSpeed | int
    ) -> bool:
        """Set collision detection speed."""
        self._current_homing_params.collision_detection_speed = CollisionDetectionSpeed(
            collision_detection_speed
        )
        SetHomeParam.unlock()
        return SetHomeParam(self.device_params, self._current_homing_params).is_success

    def set_collision_detection_current(
        self, collision_detection_current: CollisionDetectionCurrent | int
    ) -> bool:
        """Set collision detection current."""
        self._current_homing_params.collision_detection_current = CollisionDetectionCurrent(
            collision_detection_current
        )
        SetHomeParam.unlock()
        return SetHomeParam(self.device_params, self._current_homing_params).is_success

    def set_collision_detection_time(
        self, collision_detection_time: CollisionDetectionTime | int
    ) -> bool:
        """Set collision detection time."""
        self._current_homing_params.collision_detection_time = CollisionDetectionTime(
            collision_detection_time
        )
        SetHomeParam.unlock()
        return SetHomeParam(self.device_params, self._current_homing_params).is_success

    def set_auto_home(self, auto_home: AutoHoming) -> bool:
        """Set auto home."""
        self._current_homing_params.auto_home = auto_home
        SetHomeParam.unlock()
        return SetHomeParam(self.device_params, self._current_homing_params).is_success

    def home(self) -> bool:
        """Return to the home position."""
        return Home(self.device_params).is_success

    def set_home(self) -> bool:
        """Set the current position as home."""
        return SetHome(self.device_params).is_success

    def stop_home(self) -> bool:
        """Stop homing."""
        return StopHome(self.device_params).is_success

    # move related methods and properties
    def enable(self) -> bool:
        """Enable the stepper."""
        return Enable(self.device_params).is_success

    def disable(self) -> bool:
        """Disable the stepper."""
        return Disable(self.device_params).is_success

    def estop(self) -> bool:
        """Emergency stop."""
        return EStop(self.device_params).is_success

    def jog(self, jog_params: JogParams | None = None) -> bool:
        """Jog in a direction."""
        self._current_jog_params = jog_params or self._current_jog_params
        return Jog(self.device_params, self._current_jog_params).is_success

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
        if isinstance(speed, int):
            if speed < 0:
                self._current_jog_params.direction = Direction.CCW
                self._current_jog_params.speed = Speed(-speed)
            else:
                self._current_jog_params.direction = Direction.CW
                self._current_jog_params.speed = Speed(speed)
        return self.jog()

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

    def move(self, position_params: PositionParams | None = None) -> bool:
        """Move to a position."""
        self._current_position_params = position_params or self._current_position_params
        return Move(self.device_params, self._current_position_params).is_success

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

    def sync_move(self) -> bool:
        """Sync move to a position."""
        return SyncMove(self.device_params).is_success
