from dataclasses import dataclass, field
from abc import ABC, abstractmethod
from logging import getLogger
from stepper_constants import (
    Code,
    Protocol,
    EnableFlag,
    Direction,
    StoreFlag,
    SyncFlag,
    AbsoluteFlag,
    HomingMode,
    ControlMode,
    OpenLoopCurrent,
    EnablePin,
    BaudRate,
    CanRate,
    MotorType,
    CommMode,
    EnLevel,
    DefaultDir,
    SpeedReduction,
    MicrostepInterp,
    ScreenOff,
    ResponseMode,
    StallProtect,
    ChecksumMode,
    Address,
    Speed,
    Acceleration,
    PulseCount,
    Microstep,
    Kpid,
    ClosedLoopCurrent,
    MaxVoltage,
    StallSpeed,
    StallTime,
    HomingDirection,
    HomingSpeed,
    HomingTimeout,
    CollisionDetectionSpeed,
    CollisionDetectionCurrent,
    CollisionDetectionTime,
    AutoHoming,
    HomingStatus,
    PosWindow,
)
from stepper_exceptions import CommandError

logger = getLogger(__name__)


def _calculate_checksum(checksum_mode: ChecksumMode, command_bytes: bytes) -> int:
    """Calculate checksum based on selected method"""

    def _calculate_xor_checksum(data: bytes) -> int:
        """Calculate XOR checksum of bytes"""
        checksum = 0
        for byte in data:
            checksum ^= byte
        return checksum

    def _calculate_crc8(data: bytes) -> int:
        """Calculate CRC-8 with polynomial x^8 + x^2 + x + 1 (0x07)"""
        crc = 0
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = (crc << 1) ^ 0x07
                else:
                    crc <<= 1
            crc &= 0xFF
        return crc

    match checksum_mode:
        case ChecksumMode.FIXED:
            return Protocol.FIXED_CHECKSUM_BYTE
        case ChecksumMode.XOR:
            return _calculate_xor_checksum(command_bytes)
        case ChecksumMode.CRC8:
            return _calculate_crc8(command_bytes)
        case _:
            raise CommandError("Invalid checksum mode")


def _add_checksum(checksum_mode: ChecksumMode, command: bytes) -> bytes:
    checksum = _calculate_checksum(checksum_mode, command)
    return command + bytes([checksum])


def _int(input: bytes) -> int:
    return int.from_bytes(input, "big")


def _signed_int(input: bytes) -> int:
    sign = -1 if input[0] == 1 else 1
    return sign * _int(input[1:])


@dataclass
class Command(ABC):
    """Command configuration

    :param addr: Device address
    :param checksum_mode: Checksum calculation mode
    :param command_bytes: Command bytes (auto-generated)
    """

    addr: Address
    checksum_mode: ChecksumMode
    command_bytes: bytes = field(default_factory=bytes)
    _response: bytes = field(default=b"", init=False)

    def __post_init__(self):
        self.command_bytes = _add_checksum(self.checksum_mode, self.command)

    @property
    @abstractmethod
    def code(self) -> Code: ...

    @property
    @abstractmethod
    def command(self) -> bytes: ...

    @property
    def response(self) -> bytes:
        return self._response

    @response.setter
    def response(self, response: bytes):
        self._response = response

    @property
    def response_dict(self) -> dict[str, int]:
        return {
            "addr": Address(self._response[0]),
            "code": Code(self._response[1]),
            "status": self._response[2],
            "checksum": self._response[3],
        }

    def verify(self):
        """Output result of the command"""
        match self.response_dict:
            case {"code": Code.ERROR}:
                logger.error(f"Device {self.addr}: {self.code.name} failed")
                raise CommandError("Error")
            case {"status": Protocol.ERROR}:
                logger.error(
                    f"Device {self.addr}: {self.code.name} failed due to conditional error"
                )
                raise CommandError("Conditional error")
            case {"status": Protocol.SUCCESS}:
                logger.info(f"Device {self.addr}: {self.code.name} successful")
                return self.response_dict
            case _:
                raise CommandError("Invalid response")


@dataclass
class Enable(Command):
    """Enable command configuration

    :param enable_status: Enable status flag
    :param sync: Sync flag
    """

    enable_status: EnableFlag
    sync: SyncFlag

    @property
    def code(self) -> Code:
        return Code.ENABLE

    @property
    def command(self) -> bytes:
        return bytes(
            [self.addr, self.code, Protocol.ENABLE, self.enable_status, self.sync]
        )


@dataclass
class Jog(Command):
    """Jog command configuration

    :param direction: Movement direction
    :param speed: Movement speed
    :param acceleration: Movement acceleration
    :param sync: Sync flag
    """

    direction: Direction
    speed: Speed
    acceleration: Acceleration
    sync: SyncFlag

    @property
    def code(self) -> Code:
        return Code.JOG

    @property
    def command(self) -> bytes:
        return bytes(
            [
                self.addr,
                self.code,
                self.direction,
                *self.speed.bytes,
                self.acceleration,
                self.sync,
            ]
        )


@dataclass
class Move(Command):
    """Move command configuration

    :param direction: Movement direction
    :param speed: Movement speed
    :param acceleration: Movement acceleration
    :param pulse_count: Number of pulses to move
    :param mode: Relative/Absolute mode
    :param sync: Sync flag
    """

    direction: Direction
    speed: Speed
    acceleration: Acceleration
    pulse_count: PulseCount
    mode: AbsoluteFlag
    sync: SyncFlag

    @property
    def code(self) -> Code:
        return Code.MOVE

    @property
    def command(self) -> bytes:
        return bytes(
            [
                self.addr,
                self.code,
                self.direction,
                *self.speed.bytes,
                self.acceleration,
                *self.pulse_count.bytes,
                self.mode,
                self.sync,
            ]
        )


@dataclass
class EStop(Command):
    """Emergency stop command configuration

    :param sync: Sync flag
    """

    sync: SyncFlag

    @property
    def code(self) -> Code:
        return Code.ESTOP

    @property
    def command(self) -> bytes:
        return bytes([self.addr, self.code, Protocol.ESTOP, self.sync])


@dataclass
class SyncMove(Command):
    """Sync move command configuration"""

    @property
    def code(self) -> Code:
        return Code.SYNC_MOVE

    @property
    def command(self) -> bytes:
        return bytes([self.addr, self.code, Protocol.SYNC_MOVE])


@dataclass
class SetHome(Command):
    """Set home command configuration

    :param store: Store flag
    """

    store: StoreFlag

    @property
    def code(self) -> Code:
        return Code.SET_HOME

    @property
    def command(self) -> bytes:
        return bytes([self.addr, self.code, Protocol.SET_HOME, self.store])


@dataclass
class Home(Command):
    """Home command configuration

    :param homing_mode: Homing mode
    :param sync: Sync flag
    """

    homing_mode: HomingMode
    sync: SyncFlag

    @property
    def code(self) -> Code:
        return Code.HOME

    @property
    def command(self) -> bytes:
        return bytes([self.addr, self.code, self.homing_mode, self.sync])


@dataclass
class StopHome(Command):
    """Stop homing command configuration"""

    @property
    def code(self) -> Code:
        return Code.STOP_HOME

    @property
    def command(self) -> bytes:
        return bytes([self.addr, self.code, Protocol.STOP_HOME])


@dataclass
class GetHomeParam(Command):
    """Get home parameters command configuration"""

    @property
    def code(self) -> Code:
        return Code.GET_HOME_PARAM

    @property
    def command(self) -> bytes:
        return bytes([self.addr, self.code])

    @property
    def response_dict(self) -> dict[str, int]:
        response = self.response
        return {
            "addr": Address(response[0]),
            "code": Code(response[1]),
            "homing_mode": HomingMode(response[2]),
            "homing_direction": HomingDirection(response[3]),
            "homing_speed": HomingSpeed(_int(response[4:6])),
            "homing_timeout": HomingTimeout(_int(response[6:10])),
            "collision_detection_speed": CollisionDetectionSpeed(_int(response[10:12])),
            "collision_detection_current": CollisionDetectionCurrent(
                _int(response[12:14])
            ),
            "collision_detection_time": CollisionDetectionTime(_int(response[14:16])),
            "auto_home": AutoHoming(response[16]),
            "checksum": response[17],
        }


@dataclass
class SetHomeParam(Command):
    """Set home parameters command configuration

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

    @property
    def code(self) -> Code:
        return Code.SET_HOME_PARAM

    @property
    def command(self) -> bytes:
        return bytes(
            [
                self.addr,
                self.code,
                Protocol.SET_HOME_PARAM,
                self.store,
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


@dataclass
class GetHomeStatus(Command):
    """Get home status command configuration"""

    @property
    def code(self) -> Code:
        return Code.GET_HOME_STATUS

    @property
    def command(self) -> bytes:
        return bytes([self.addr, self.code])

    @property
    def response_dict(self) -> dict[str, int]:
        response = self.response
        return {
            "addr": Address(response[0]),
            "code": Code(response[1]),
            "homing_status": HomingStatus(response[2]),
            "checksum": response[3],
        }


@dataclass
class CalEncoder(Command):
    """Calibrate encoder command configuration"""

    @property
    def code(self) -> Code:
        return Code.CAL_ENCODER

    @property
    def command(self) -> bytes:
        return bytes([self.addr, self.code, Protocol.CAL_ENCODER])


@dataclass
class ClearPos(Command):
    """Clear position command configuration"""

    @property
    def code(self) -> Code:
        return Code.CLEAR_POS

    @property
    def command(self) -> bytes:
        return bytes([self.addr, self.code, Protocol.CLEAR_POS])


@dataclass
class ClearStallCommand(Command):
    """Clear stall command configuration"""

    @property
    def code(self) -> Code:
        return Code.CLEAR_STALL

    @property
    def command(self) -> bytes:
        return bytes([self.addr, self.code, Protocol.CLEAR_STALL])


@dataclass
class ResetCommand(Command):
    """Reset command configuration"""

    @property
    def code(self) -> Code:
        return Code.RESET

    @property
    def command(self) -> bytes:
        return bytes([self.addr, self.code, Protocol.RESET])


@dataclass
class GetVersion(Command):
    """Get version command configuration"""

    @property
    def code(self) -> Code:
        return Code.GET_VERSION

    @property
    def command(self) -> bytes:
        return bytes([self.addr, self.code])

    @property
    def response_dict(self, response: bytes) -> dict[str, int]:
        return {
            "addr": response[0],
            "code": response[1],
            "firmware_version": response[2],
            "hardware_version": response[3],
            "checksum": response[4],
        }


@dataclass
class GetMotorParam(Command):
    """Get motor parameters command configuration"""

    @property
    def code(self) -> Code:
        return Code.GET_MOTOR_PARAM

    @property
    def command(self) -> bytes:
        return bytes([self.addr, self.code])

    @property
    def response_dict(self) -> dict[str, int]:
        response = self.response
        return {
            "addr": response[0],
            "code": response[1],
            "phase_resistance": _int(response[2:4]),
            "phase_inductance": _int(response[4:6]),
            "checksum": response[6],
        }


@dataclass
class GetPID(Command):
    """Get PID parameters command configuration"""

    @property
    def code(self) -> Code:
        return Code.GET_PID

    @property
    def command(self) -> bytes:
        return bytes([self.addr, self.code])

    @property
    def response_dict(self) -> dict[str, int]:
        response = self.response
        return {
            "addr": response[0],
            "code": response[1],
            "kp": _int(response[2:6]),
            "ki": _int(response[6:10]),
            "kd": _int(response[10:14]),
            "checksum": response[14],
        }


@dataclass
class GetVoltage(Command):
    """Get voltage command configuration"""

    @property
    def code(self) -> Code:
        return Code.GET_VOLTAGE

    @property
    def command(self) -> bytes:
        return bytes([self.addr, self.code])

    @property
    def response_dict(self) -> dict[str, int]:
        response = self.response
        return {
            "addr": response[0],
            "code": response[1],
            "bus_voltage": _int(response[2:4]),
            "checksum": response[4],
        }


@dataclass
class GetCurrent(Command):
    """Get current command configuration"""

    @property
    def code(self) -> Code:
        return Code.GET_CURRENT

    @property
    def command(self) -> bytes:
        return bytes([self.addr, self.code])

    @property
    def response_dict(self) -> dict[str, int]:
        response = self.response
        return {
            "addr": response[0],
            "code": response[1],
            "bus_phase_current": _int(response[2:4]),
            "checksum": response[4],
        }


@dataclass
class GetEncoder(Command):
    """Get encoder value command configuration"""

    @property
    def code(self) -> Code:
        return Code.GET_ENCODER

    @property
    def command(self) -> bytes:
        return bytes([self.addr, self.code])

    @property
    def response_dict(self) -> dict[str, int]:
        response = self.response
        return {
            "addr": response[0],
            "code": response[1],
            "calibrated_encoder_value": _int(response[2:4]),
            "checksum": response[4],
        }


@dataclass
class GetPulse(Command):
    """Get pulse count command configuration"""

    @property
    def code(self) -> Code:
        return Code.GET_PULSE

    @property
    def command(self) -> bytes:
        return bytes([self.addr, self.code])

    @property
    def response_dict(self) -> dict[str, int]:
        response = self.response
        return {
            "addr": response[0],
            "code": response[1],
            "input_pulse_count": _signed_int(response[2:7]),
            "checksum": response[7],
        }


@dataclass
class GetTarget(Command):
    """Get target position command configuration"""

    @property
    def code(self) -> Code:
        return Code.GET_TARGET

    @property
    def command(self) -> bytes:
        return bytes([self.addr, self.code])

    @property
    def response_dict(self) -> dict[str, int]:
        response = self.response
        return {
            "addr": response[0],
            "code": response[1],
            "motor_target_position": _signed_int(response[2:7]),
            "checksum": response[7],
        }


@dataclass
class GetSetpoint(Command):
    """Get setpoint command configuration"""

    @property
    def code(self) -> Code:
        return Code.GET_SETPOINT

    @property
    def command(self) -> bytes:
        return bytes([self.addr, self.code])

    @property
    def response_dict(self) -> dict[str, int]:
        response = self.response
        return {
            "addr": response[0],
            "code": response[1],
            "motor_target_position": _signed_int(response[2:7]),
            "checksum": response[7],
        }


@dataclass
class GetSpeed(Command):
    """Get speed command configuration"""

    @property
    def code(self) -> Code:
        return Code.GET_SPEED

    @property
    def command(self) -> bytes:
        return bytes([self.addr, self.code])

    @property
    def response_dict(self) -> dict[str, int]:
        response = self.response
        return {
            "addr": response[0],
            "code": response[1],
            "motor_real_time_speed": _signed_int(response[2:5]),
            "checksum": response[5],
        }


@dataclass
class GetPos(Command):
    """Get position command configuration"""

    @property
    def code(self) -> Code:
        return Code.GET_POS

    @property
    def command(self) -> bytes:
        return bytes([self.addr, self.code])

    @property
    def response_dict(self) -> dict[str, int]:
        response = self.response
        return {
            "addr": response[0],
            "code": response[1],
            "motor_real_time_position": _signed_int(response[2:7]),
            "checksum": response[7],
        }


@dataclass
class GetError(Command):
    """Get error command configuration"""

    @property
    def code(self) -> Code:
        return Code.GET_ERROR

    @property
    def command(self) -> bytes:
        return bytes([self.addr, self.code])

    @property
    def response_dict(self) -> dict[str, int]:
        response = self.response
        return {
            "addr": response[0],
            "code": response[1],
            "motor_position_error": _signed_int(response[2:7]),
            "checksum": response[7],
        }


@dataclass
class GetStatus(Command):
    """Get status command configuration"""

    @property
    def code(self) -> Code:
        return Code.GET_STATUS

    @property
    def command(self) -> bytes:
        return bytes([self.addr, self.code])


@dataclass
class SetMicrostep(Command):
    """Set microstep command configuration

    :param store: Store flag
    :param microstep_value: Microstep value
    """

    store: StoreFlag
    microstep_value: Microstep

    @property
    def code(self) -> Code:
        return Code.SET_MICROSTEP

    @property
    def command(self) -> bytes:
        return bytes(
            [
                self.addr,
                self.code,
                Protocol.SET_MICROSTEP,
                self.store,
                self.microstep_value,
            ]
        )


@dataclass
class SetID(Command):
    """Set ID command configuration

    :param store: Store flag
    :param id_addr: ID address
    """

    store: StoreFlag
    id_addr: Address

    @property
    def code(self) -> Code:
        return Code.SET_ID

    @property
    def command(self) -> bytes:
        return bytes([self.addr, self.code, Protocol.SET_ID, self.store, self.id_addr])


@dataclass
class SetMode(Command):
    """Set mode command configuration

    :param store: Store flag
    :param control_mode: Control mode
    """

    store: StoreFlag
    control_mode: ControlMode

    @property
    def code(self) -> Code:
        return Code.SET_MODE

    @property
    def command(self) -> bytes:
        return bytes(
            [self.addr, self.code, Protocol.SET_MODE, self.store, self.control_mode]
        )


@dataclass
class SetCurrent(Command):
    """Set current command configuration

    :param store: Store flag
    :param open_loop_current: Open loop current
    """

    store: StoreFlag
    open_loop_current: OpenLoopCurrent

    @property
    def code(self) -> Code:
        return Code.SET_CURRENT

    @property
    def command(self) -> bytes:
        return bytes(
            [
                self.addr,
                self.code,
                Protocol.SET_CURRENT,
                self.store,
                *self.open_loop_current.bytes,
            ]
        )


@dataclass
class SetPID(Command):
    """Set PID parameters command configuration

    :param store: Store flag
    :param kp: Proportional gain
    :param ki: Integral gain
    :param kd: Derivative gain
    """

    store: StoreFlag
    kp: Kpid
    ki: Kpid
    kd: Kpid

    @property
    def code(self) -> Code:
        return Code.SET_PID

    @property
    def command(self) -> bytes:
        return bytes(
            [
                self.addr,
                self.code,
                Protocol.SET_PID,
                self.store,
                *self.kp.bytes,
                *self.ki.bytes,
                *self.kd.bytes,
            ]
        )


@dataclass
class SaveSpeed(Command):
    """Save speed command configuration

    :param store: Store flag
    :param direction: Direction
    :param speed: Speed
    :param acceleration: Acceleration
    :param en_control: Enable control
    """

    store: StoreFlag
    direction: Direction
    speed: Speed
    acceleration: Acceleration
    en_control: EnablePin

    @property
    def code(self) -> Code:
        return Code.SAVE_SPEED

    @property
    def command(self) -> bytes:
        return bytes(
            [
                self.addr,
                self.code,
                Protocol.SAVE_SPEED,
                self.store,
                self.direction,
                *self.speed.bytes,
                *self.acceleration.bytes,
                self.en_control,
            ]
        )


@dataclass
class SetReduction(Command):
    """Set speed reduction command configuration

    :param store: Store flag
    :param speed_reduction: Speed reduction
    """

    store: StoreFlag
    speed_reduction: SpeedReduction

    @property
    def code(self) -> Code:
        return Code.SET_REDUCTION

    @property
    def command(self) -> bytes:
        return bytes(
            [
                self.addr,
                self.code,
                Protocol.SET_REDUCTION,
                self.store,
                self.speed_reduction,
            ]
        )


@dataclass
class GetConfig(Command):
    """Get configuration command configuration"""

    @property
    def code(self) -> Code:
        return Code.GET_CONFIG

    @property
    def command(self) -> bytes:
        return bytes([self.addr, self.code, Protocol.GET_CONFIG])

    @property
    def response_dict(self) -> dict[str, int]:
        response = self.response
        return {
            "addr": response[0],
            "code": response[1],
            "byte_length": response[2],
            "param_count": response[3],
            "motor_type": response[4],
            "control_mode": response[5],
            "comm_mode": response[6],
            "en_level": response[7],
            "dir_level": response[8],
            "microsteps": response[9],
            "microstep_interp": response[10],
            "screen_off": response[11],
            "open_loop_current": _int(response[12:14]),
            "closed_loop_current": _int(response[14:16]),
            "max_voltage": _int(response[16:18]),
            "baud_rate": response[18],
            "can_rate": response[19],
            "id_addr": response[20],
            "verify_mode": response[21],
            "response_mode": response[22],
            "stall_protect": response[23],
            "stall_speed": _int(response[24:26]),
            "stall_current": _int(response[26:28]),
            "stall_time": _int(response[28:30]),
            "pos_window": _int(response[30:32]),
            "checksum": response[32],
        }


@dataclass
class SetConfig(Command):
    """Set configuration command configuration

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
    :param id_addr: ID address
    :param verify_mode: Verify mode
    :param response_mode: Response mode
    :param stall_protect: Stall protection
    :param stall_speed: Stall speed
    :param stall_current: Stall current
    :param stall_time: Stall time
    :param pos_window: Position window
    """

    store: StoreFlag
    motor_type: MotorType
    control_mode: ControlMode
    comm_mode: CommMode
    en_level: EnLevel
    dir_level: DefaultDir
    microsteps: Microstep
    microstep_interp: MicrostepInterp
    screen_off: ScreenOff
    open_loop_current: OpenLoopCurrent
    closed_loop_current: ClosedLoopCurrent
    max_voltage: MaxVoltage
    baud_rate: BaudRate
    can_rate: CanRate
    id_addr: Address  # Not implemented
    checksum_mode: ChecksumMode
    response_mode: ResponseMode
    stall_protect: StallProtect
    stall_speed: StallSpeed
    stall_current: ClosedLoopCurrent
    stall_time: StallTime
    pos_window: PosWindow

    @property
    def code(self) -> Code:
        return Code.SET_CONFIG

    @property
    def command(self) -> bytes:
        return bytes(
            [
                self.addr,
                self.code,
                Protocol.SET_CONFIG,
                self.store,
                self.motor_type,
                self.control_mode,
                self.comm_mode,
                self.en_level,
                self.dir_level,
                self.microsteps,
                self.microstep_interp,
                self.screen_off,
                *self.open_loop_current.bytes,
                *self.closed_loop_current.bytes,
                *self.max_voltage.bytes,
                self.baud_rate,
                self.can_rate,
                self.id_addr,
                self.checksum_mode,
                self.response_mode,
                self.stall_protect,
                *self.stall_speed.bytes,
                *self.stall_current.bytes,
                *self.stall_time.bytes,
                *self.pos_window.bytes,
            ]
        )


@dataclass
class GetSysStatus(Command):
    """Get system status command configuration"""

    @property
    def code(self) -> Code:
        return Code.GET_SYS_STATUS

    @property
    def command(self) -> bytes:
        return bytes([self.addr, self.code, Protocol.GET_SYS_STATUS])

    @property
    def response_dict(self) -> dict[str, int]:
        response = self.response
        return {
            "addr": response[0],
            "code": response[1],
            "protocol_1": response[2],
            "protocol_2": response[3],
            "bus_voltage": _int(response[4:6]),
            "bus_phase_current": _int(response[6:8]),
            "calibrated_encoder_value": _int(response[8:10]),
            "motor_target_position": _signed_int(response[10:15]),
            "motor_real_time_speed": _signed_int(response[15:18]),
            "motor_real_time_position": _signed_int(response[18:23]),
            "motor_position_error": _signed_int(response[23:28]),
            "ready_status": response[28],
            "motor_status": response[29],
            "checksum": response[30],
        }
