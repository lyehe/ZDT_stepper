"""Get commands for stepper motor."""

from dataclasses import asdict, dataclass
from logging import getLogger

from src.stepper.stepper_command import (
    ReturnData,
    TakeNoSetting,
    WithNoParams,
    to_int,
)
from src.stepper.stepper_constants import (
    Address,
    AnglePosition,
    AngleUnit,
    BaudRate,
    CanRate,
    ChecksumMode,
    Code,
    CommunicationMode,
    ControlMode,
    CurrentUnit,
    Direction,
    EnableLevel,
    HomingStatus,
    Kpid,
    Microstep,
    MicrostepInterp,
    MotorType,
    Protocol,
    ResponseMode,
    ScreenOff,
    StallProtect,
    StepperStatus,
    TimeUnit,
    VoltageUnit,
)
from src.stepper.stepper_parameters import (
    CurrentParams,
    EncoderParams,
    MotorRHParams,
    PIDParams,
    VersionParams,
    VoltageParams,
)

logger = getLogger(__name__)


class GetCommand(WithNoParams, TakeNoSetting):
    """Get command configuration."""


@dataclass
class GetVersion(GetCommand, ReturnData):
    """Get version of the device."""

    _code = Code.GET_VERSION
    _response_length = 5
    ReturnType = VersionParams

    def _unpack_data(self, data: bytes) -> VersionParams:
        """Return home parameters as a dictionary."""
        return VersionParams(
            firmware_version=data[0],
            hardware_version=data[1],
        )


@dataclass
class GetMotorRH(GetCommand, ReturnData):
    """Get motor resistance and inductance."""

    _code = Code.GET_MOTOR_R_H
    _response_length = 7
    ReturnType = MotorRHParams

    def _unpack_data(self, data: bytes) -> MotorRHParams:
        """Return motor resistance and inductance as a dictionary."""
        return MotorRHParams(
            phase_resistance=to_int(data[0:2]),
            phase_inductance=to_int(data[2:4]),
        )


@dataclass
class GetPID(GetCommand, ReturnData):
    """Get PID parameters command configuration."""

    _code = Code.GET_PID
    _response_length = 15
    ReturnType = PIDParams

    def _unpack_data(self, data: bytes) -> PIDParams:
        """Return PID parameters as a dictionary."""
        return PIDParams(
            pid_p=Kpid(to_int(data[0:4])),
            pid_i=Kpid(to_int(data[4:8])),
            pid_d=Kpid(to_int(data[8:12])),
        )


@dataclass
class GetBusVoltage(GetCommand, ReturnData):
    """Get the bus voltage."""

    _code = Code.GET_BUS_VOLTAGE
    _response_length = 5
    ReturnType = VoltageParams

    def _unpack_data(self, data: bytes) -> VoltageParams:
        """Return bus voltage as a dictionary."""
        return VoltageParams(
            voltage=to_int(data),
        )


@dataclass
class GetPhaseCurrent(GetCommand):
    """Get phase current."""

    _code = Code.GET_PHASE_CURRENT
    _response_length = 5
    ReturnType = CurrentParams

    def _unpack_data(self, data: bytes) -> CurrentParams:
        """Return phase current as a dictionary."""
        return CurrentParams(
            current=to_int(data),
        )


@dataclass
class GetEncoderValue(AnglePosition):
    """Get encoder value."""

    _code = Code.GET_ENCODER_VALUE
    _response_length = 5
    ReturnType = EncoderParams

    def _unpack_data(self, data: bytes) -> EncoderParams:
        """Return encoder value as a dictionary."""
        return EncoderParams(
            encoder_value=to_int(data),
        )


@dataclass
class GetPulseCount(GetCommand):
    """Get pulse count."""

    _code = Code.GET_PULSE_COUNT
    _response_length = 7
    ReturnType = PulseCountParams

    def _unpack_data(self, data: bytes) -> PulseCountParams:
        """Return pulse count as a dictionary."""
        return PulseCountParams(
            pulse_count=to_int(data),
        )

    @property
    def _command_body(self) -> bytes:
        """Command bytes."""
        return bytes([self.address, self._code])

    @property
    def response_dict(self) -> dict[str, int]:
        """Response dictionary."""
        response = self.response
        return {
            "address": Address(response[0]),
            "code": Code(response[1]).name,
            "pulse_count": _signed_int(response[2:7]),
            "checksum": response[7],
        }

    @property
    def raw_value(self) -> int:
        return _signed_int(self.response[2:7])

    @property
    def pulse_count(self) -> int:
        return self.raw_value

    @property
    def value(self) -> int:
        return self.pulse_count

    @property
    def parameter_dict(self) -> dict[str, int]:
        return {
            "pulse_count": self.pulse_count,
        }


@dataclass
class GetTarget(AnglePosition):
    """Get target position"""

    @property
    def _code(self) -> Code:
        return Code.GET_TARGET

    @property
    def _command_body(self) -> bytes:
        return bytes([self.address, self._code])

    @property
    def response_dict(self) -> dict[str, int]:
        response = self.response
        return {
            "address": Address(response[0]),
            "code": Code(response[1]).name,
            "motor_target_position": _signed_int(response[2:7]),
            "checksum": response[7],
        }

    @property
    def raw_value(self) -> int:
        return _signed_int(self.response[2:7])

    @property
    def value(self) -> int:
        return self.angle

    @property
    def parameter_dict(self) -> dict[str, float]:
        return {
            f"target_position ({self.angle_unit.name})": self.angle,
            "target_turns": self.turns,
            f"remainder ({self.angle_unit.name})": self.angle_remainder,
        }


@dataclass
class GetOpenLoopSetpoint(AnglePosition):
    """Get open loop setpoint"""

    @property
    def _code(self) -> Code:
        return Code.GET_OPEN_LOOP_SETPOINT

    @property
    def _command_body(self) -> bytes:
        return bytes([self.address, self._code])

    @property
    def response_dict(self) -> dict[str, int]:
        response = self.response
        return {
            "address": Address(response[0]),
            "code": Code(response[1]).name,
            "motor_open_loop_setpoint": _signed_int(response[2:7]),
            "checksum": response[7],
        }

    @property
    def raw_value(self) -> int:
        return _signed_int(self.response[2:7])

    @property
    def value(self) -> int:
        return self.angle

    @property
    def parameter_dict(self) -> dict[str, float]:
        return {
            f"setpoint ({self.angle_unit.name})": self.angle,
            "setpoint_turns": self.turns,
            f"remainder ({self.angle_unit.name})": self.angle_remainder,
        }


@dataclass
class GetSpeed(GetCommand):
    """Get real time speed in RPM"""

    @property
    def _code(self) -> Code:
        return Code.GET_SPEED

    @property
    def _command_body(self) -> bytes:
        return bytes([self.address, self._code])

    @property
    def response_dict(self) -> dict[str, int]:
        response = self.response
        return {
            "address": Address(response[0]),
            "code": Code(response[1]).name,
            "motor_real_time_speed": _signed_int(response[2:5]),
            "checksum": response[5],
        }

    @property
    def raw_value(self) -> int:
        return _signed_int(self.response[2:5])

    @property
    def speed(self) -> int:
        return self.raw_value

    @property
    def value(self) -> int:
        return self.speed

    @property
    def parameter_dict(self) -> dict[str, int]:
        return {
            "motor_real_time_speed (RPM)": self.speed,
        }


@dataclass
class GetPosition(AnglePosition):
    """Get position command configuration"""

    @property
    def _code(self) -> Code:
        return Code.GET_POS

    @property
    def _command_body(self) -> bytes:
        return bytes([self.address, self._code])

    @property
    def response_dict(self) -> dict[str, int]:
        response = self.response
        return {
            "address": Address(response[0]),
            "code": Code(response[1]).name,
            "motor_real_time_position": _signed_int(response[2:7]),
            "checksum": response[7],
        }

    @property
    def raw_value(self) -> int:
        return _signed_int(self.response[2:7])

    @property
    def position(self) -> int:
        return self.raw_value

    @property
    def value(self) -> int:
        return self.position

    @property
    def parameter_dict(self) -> dict[str, float]:
        return {
            f"motor_real_time_position ({self.angle_unit.name})": self.angle,
            "motor_real_time_turns": self.turns,
            f"motor_real_time_remainder ({self.angle_unit.name})": self.angle_remainder,
        }


@dataclass
class GetError(AnglePosition):
    """Get error command configuration"""

    @property
    def _code(self) -> Code:
        return Code.GET_ERROR

    @property
    def _command_body(self) -> bytes:
        return bytes([self.address, self._code])

    @property
    def response_dict(self) -> dict[str, int]:
        response = self.response
        return {
            "address": Address(response[0]),
            "code": Code(response[1]).name,
            "motor_position_error": _signed_int(response[2:7]),
            "checksum": response[7],
        }

    @property
    def raw_value(self) -> int:
        return _signed_int(self.response[2:7])

    @property
    def error(self) -> int:
        return self.raw_value

    @property
    def value(self) -> int:
        return self.error

    @property
    def parameter_dict(self) -> dict[str, float]:
        return {
            f"motor_position_error ({self.angle_unit.name})": self.angle,
            "motor_position_error_turns": self.turns,
            f"motor_position_error_remainder ({self.angle_unit.name})": self.angle_remainder,
        }


@dataclass
class GetStatus(GetCommand):
    """Get status of the stepper motor"""

    @property
    def _code(self) -> Code:
        return Code.GET_STATUS

    @property
    def _command_body(self) -> bytes:
        return bytes([self.address, self._code])

    @property
    def response_dict(self) -> dict[str, int]:
        response = self.response
        return {
            "address": Address(response[0]),
            "code": Code(response[1]).name,
            "ready_status": StepperStatus(response[2]),
            "checksum": response[3],
        }

    @property
    def raw_value(self) -> int:
        return self.response[2]

    @property
    def status(self) -> StepperStatus:
        return StepperStatus(self.raw_value)

    @property
    def value(self) -> StepperStatus:
        return self.status

    @property
    def parameter_dict(self) -> dict[str, bool]:
        return asdict(self.status)

    @property
    def is_enabled(self) -> bool:
        return self.status.enabled

    @property
    def is_in_position(self) -> bool:
        return self.status.in_position

    @property
    def is_stalled(self) -> bool:
        return self.status.stalled

    @property
    def is_stall_protection_active(self) -> bool:
        return self.status.stall_protection_active


@dataclass
class GetConfig(GetCommand):
    """Get configuration"""

    current_unit: CurrentUnit = CurrentUnit.default
    voltage_unit: VoltageUnit = VoltageUnit.default
    time_unit: TimeUnit = TimeUnit.default

    @property
    def _code(self) -> Code:
        return Code.GET_CONFIG

    @property
    def _command_body(self) -> bytes:
        return bytes([self.address, self._code, Protocol.GET_CONFIG])

    @property
    def response_dict(self) -> dict[str, int]:
        response = self.response
        return {
            "address": Address(response[0]),
            "code": Code(response[1]),
            "byte_length": response[2],
            "param_count": response[3],
            "motor_type": MotorType(response[4]),
            "control_mode": ControlMode(response[5]),
            "communication_mode": CommunicationMode(response[6]),
            "enable_level": EnableLevel(response[7]),
            "default_direction": Direction(response[8]),
            "microsteps": Microstep(response[9]),
            "microstep_interp": MicrostepInterp(response[10]),
            "screen_off": ScreenOff(response[11]),
            "open_loop_current": _int(response[12:14]),
            "max_closed_loop_current": _int(response[14:16]),
            "max_voltage": _int(response[16:18]),
            "baud_rate": BaudRate(response[18]),
            "can_rate": CanRate(response[19]),
            "address": Address(response[20]),
            "checksum_mode": ChecksumMode(response[21]),
            "response_mode": ResponseMode(response[22]),
            "stall_protect": StallProtect(response[23]),
            "stall_speed": _int(response[24:26]),
            "stall_current": _int(response[26:28]),
            "stall_time": _int(response[28:30]),
            "on_target_window": _int(response[30:32]),  # 0.1XDeg
            "checksum": response[32],
        }

    @property
    def raw_value(
        self,
    ) -> tuple[int, ...]:
        response = self.response_dict.copy()
        response.pop("checksum")
        response.pop("address")
        response.pop("code")
        response.pop("byte_length")
        response.pop("param_count")
        return tuple(response.values())

    @property
    def parameter_dict(self) -> dict[str, float | str]:
        response = self.response
        return {
            "motor_type": MotorType(response[4]).name,
            "control_mode": ControlMode(response[5]).name,
            "communication_mode": CommunicationMode(response[6]).name,
            "enable_level": EnableLevel(response[7]).name,
            "default_direction": Direction(response[8]).name,
            "microsteps": Microstep(response[9]),
            "microstep_interp": MicrostepInterp(response[10]).name,
            "screen_off": ScreenOff(response[11]).name,
            f"open_loop_current ({self.current_unit.name})": _int(response[12:14])
            / self.current_unit.value,
            f"max_closed_loop_current ({self.current_unit.name})": _int(response[14:16])
            / self.current_unit.value,
            f"max_voltage ({self.voltage_unit.name})": _int(response[16:18])
            / self.voltage_unit.value,
            "baud_rate": BaudRate(response[18]).name,
            "can_rate": CanRate(response[19]).name,
            "address": Address(response[20]),
            "checksum_mode": ChecksumMode(response[21]).name,
            "response_mode": ResponseMode(response[22]).name,
            "stall_protect": StallProtect(response[23]).name,
            "stall_speed (RPM)": _int(response[24:26]),
            f"stall_current ({self.current_unit.name})": _int(response[26:28])
            / self.current_unit.value,
            f"stall_time ({self.time_unit.name})": _int(response[28:30]) / self.time_unit.value,
            "on_target_window (deg)": _int(response[30:32]) * 0.1,
        }

    @property
    def value(self) -> tuple[float | str, ...]:
        return tuple(self.parameter_dict.values())


@dataclass
class GetSysStatus(GetCommand):
    """Get system status command configuration"""

    voltage_unit: VoltageUnit = VoltageUnit.default
    current_unit: CurrentUnit = CurrentUnit.default
    angle_unit: AngleUnit = AngleUnit.default

    @property
    def _code(self) -> Code:
        return Code.GET_SYS_STATUS

    @property
    def _command_body(self) -> bytes:
        return bytes([self.address, self._code, Protocol.GET_SYS_STATUS])

    @property
    def response_dict(self) -> dict[str, int]:
        response = self.response
        return {
            "address": Address(response[0]),
            "code": Code(response[1]),
            "byte_length": response[2],
            "param_count": response[3],
            "bus_voltage": _int(response[4:6]),
            "bus_phase_current": _int(response[6:8]),
            "calibrated_encoder_value": _int(response[8:10]),
            "motor_target_position": _signed_int(response[10:15]),
            "motor_real_time_speed": _signed_int(response[15:18]),
            "motor_real_time_position": _signed_int(response[18:23]),
            "motor_position_error": _signed_int(response[23:28]),
            "homing_status": HomingStatus(response[28]),
            "motor_status": StepperStatus(response[29]),
            "checksum": response[30],
        }

    @property
    def raw_value(self) -> tuple[int, ...]:
        response = self.response_dict.copy()
        response.pop("checksum")
        response.pop("address")
        response.pop("code")
        response.pop("byte_length")
        response.pop("param_count")
        return tuple(response.values())

    @property
    def parameter_dict(self) -> dict[str, float | bool]:
        response = self.response_dict.copy()
        homing_status = HomingStatus(response[28])
        stepper_status = StepperStatus(response[29])

        def _angle(angle: int) -> float:
            return _signed_int(angle) / 65536 * self.angle_unit.value

        return {
            f"bus_voltage ({self.voltage_unit.name})": _int(response[4:6])
            / self.voltage_unit.value,
            f"bus_phase_current ({self.current_unit.name})": _int(response[6:8])
            / self.current_unit.value,
            f"calibrated_encoder_value ({self.angle_unit.name})": _int(response[8:10])
            / 65536
            * self.angle_unit.value,
            f"motor_target_position ({self.angle_unit.name})": _angle(response[10:15]),
            "motor_real_time_speed (RPM)": _signed_int(response[15:18]),
            f"motor_real_time_position ({self.angle_unit.name})": _angle(response[18:23]),
            f"motor_position_error ({self.angle_unit.name})": _angle(response[23:28]),
            "homing_encoder_ready": homing_status.encoder_ready,
            "homing_encoder_calibrated": homing_status.encoder_calibrated,
            "homing_active": homing_status.is_homing,
            "homing_failed": homing_status.homing_failed,
            "stepper_enabled": stepper_status.enabled,
            "stepper_in_position": stepper_status.in_position,
            "stepper_stalled": stepper_status.stalled,
            "stepper_stall_protection_active": stepper_status.stall_protection_active,
        }

    @property
    def value(self) -> tuple[float | bool, ...]:
        return tuple(self.parameter_dict.values())
