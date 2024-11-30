from abc import ABC, abstractmethod
from dataclasses import dataclass, asdict
from logging import getLogger
from stepper_constants import (
    Address,
    Code,
    Protocol,
    BaudRate,
    CanRate,
    ChecksumMode,
    ResponseMode,
    StallProtect,
    StepperStatus,
    MotorType,
    ControlMode,
    CommunicationMode,
    EnableLevel,
    Direction,
    Microstep,
    MicrostepInterp,
    ScreenOff,
    HomingStatus,
)
from stepper_command import Command, _int, _signed_int
import math

logger = getLogger(__name__)


@dataclass
class Position(Command, ABC):
    """Position command configuration"""

    @property
    @abstractmethod
    def value(self) -> int: ...

    @property
    def deg(self) -> float:
        return self.value / 65536 * 360

    @property
    def rad(self) -> float:
        return self.value / 65536 * 2 * math.pi

    @property
    def turns(self) -> int:
        return self.value // 65536

    @property
    def remainder_deg(self) -> float:
        return self.value % 65536 / 65536 * 360

    @property
    def remainder_rad(self) -> float:
        return self.value % 65536 / 65536 * 2 * math.pi


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
    def response_dict(self) -> dict[str, int]:
        response = self.response
        return {
            "addr": Address(response[0]),
            "code": Code(response[1]).name,
            "firmware_version": response[2],
            "hardware_version": response[3],
            "checksum": response[4],
        }

    @property
    def firmware_version(self) -> str:
        return f"FV{self.response_dict['firmware_version']}"

    @property
    def hardware_version(self) -> str:
        return f"HV{self.response_dict['hardware_version']}"

    @property
    def values(self) -> tuple[str, str]:
        return self.firmware_version, self.hardware_version

    @property
    def parameter_dict(self) -> dict[str, str]:
        return {
            "firmware_version": self.firmware_version,
            "hardware_version": self.hardware_version,
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
            "addr": Address(response[0]),
            "code": Code(response[1]).name,
            "phase_resistance": _int(response[2:4]),  # mOhm
            "phase_inductance": _int(response[4:6]),  # uH
            "checksum": response[6],
        }

    @property
    def resistance(self) -> float:
        return self.response_dict["phase_resistance"] * 0.001

    @property
    def inductance(self) -> float:
        return self.response_dict["phase_inductance"] * (1e-6)

    @property
    def values(self) -> tuple[float, float]:
        return self.resistance, self.inductance

    @property
    def parameter_dict(self) -> dict[str, float]:
        return {
            "phase_resistance (Ohm)": self.resistance,
            "phase_inductance (H)": self.inductance,
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
            "addr": Address(response[0]),
            "code": Code(response[1]).name,
            "kp": _int(response[2:6]),
            "ki": _int(response[6:10]),
            "kd": _int(response[10:14]),
            "checksum": response[14],
        }

    @property
    def kp(self) -> float:
        return self.response_dict["kp"] * 0.001

    @property
    def ki(self) -> float:
        return self.response_dict["ki"] * 0.001

    @property
    def kd(self) -> float:
        return self.response_dict["kd"] * 0.001

    @property
    def values(self) -> tuple[float, float, float]:
        return self.kp, self.ki, self.kd

    @property
    def parameter_dict(self) -> dict[str, float]:
        return {
            "kp (V/rad/s)": self.kp,
            "ki (V/rad/s^2)": self.ki,
            "kd (V/rad/s^3)": self.kd,
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
            "addr": Address(response[0]),
            "code": Code(response[1]).name,
            "bus_voltage": _int(response[2:4]),  # mV
            "checksum": response[4],
        }

    @property
    def bus_voltage(self) -> float:
        return self.response_dict["bus_voltage"] * 0.001

    @property
    def value(self) -> float:
        return self.bus_voltage

    @property
    def parameter_dict(self) -> dict[str, float]:
        return {
            "bus_voltage (V)": self.voltage,
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
            "addr": Address(response[0]),
            "code": Code(response[1]).name,
            "phase_current": _int(response[2:4]),  # mA
            "checksum": response[4],
        }

    @property
    def phase_current(self) -> float:
        return self.response_dict["phase_current"] * 0.001

    @property
    def value(self) -> float:
        return self.phase_current

    @property
    def parameter_dict(self) -> dict[str, float]:
        return {
            "phase_current (A)": self.current,
        }


@dataclass
class GetEncoder(Position):
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
            "addr": Address(response[0]),
            "code": Code(response[1]).name,
            "calibrated_encoder_value": _int(response[2:4]),
            "checksum": response[4],
        }

    @property
    def calibrated_encoder_value(self) -> int:
        return self.response_dict["calibrated_encoder_value"]

    @property
    def value(self) -> int:
        return self.calibrated_encoder_value

    @property
    def parameter_dict(self) -> dict[str, float]:
        return {
            "encoder (deg)": self.deg,
            "encoder (rad)": self.rad,
            "turns": self.turns,
        }


@dataclass
class GetPulseCount(Command):
    """Get pulse count command configuration"""

    @property
    def code(self) -> Code:
        return Code.GET_PULSE_COUNT

    @property
    def command(self) -> bytes:
        return bytes([self.addr, self.code])

    @property
    def response_dict(self) -> dict[str, int]:
        response = self.response
        return {
            "addr": Address(response[0]),
            "code": Code(response[1]).name,
            "pulse_count": _signed_int(response[2:7]),
            "checksum": response[7],
        }

    @property
    def pulse_count(self) -> int:
        return self.response_dict["pulse_count"]

    @property
    def value(self) -> int:
        return self.pulse_count

    @property
    def parameter_dict(self) -> dict[str, int]:
        return {
            "pulse_count": self.pulse_count,
        }


@dataclass
class GetTarget(Position):
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
            "addr": Address(response[0]),
            "code": Code(response[1]).name,
            "motor_target_position": _signed_int(response[2:7]),
            "checksum": response[7],
        }

    @property
    def target_position(self) -> int:
        return self.response_dict["motor_target_position"]

    @property
    def value(self) -> int:
        return self.target_position

    @property
    def parameter_dict(self) -> dict[str, float]:
        return {
            "target_position (deg)": self.deg,
            "target_position (rad)": self.rad,
            "target_turns": self.turns,
        }


@dataclass
class GetOpenLoopSetpoint(Position):
    """Get open loop setpoint command configuration"""

    @property
    def code(self) -> Code:
        return Code.GET_OPEN_LOOP_SETPOINT

    @property
    def command(self) -> bytes:
        return bytes([self.addr, self.code])

    @property
    def response_dict(self) -> dict[str, int]:
        response = self.response
        return {
            "addr": Address(response[0]),
            "code": Code(response[1]).name,
            "motor_open_loop_setpoint": _signed_int(response[2:7]),
            "checksum": response[7],
        }

    @property
    def open_loop_setpoint(self) -> int:
        return self.response_dict["motor_open_loop_setpoint"]

    @property
    def value(self) -> int:
        return self.open_loop_setpoint

    @property
    def parameter_dict(self) -> dict[str, float]:
        return {
            "setpoint (deg)": self.deg,
            "setpoint (rad)": self.rad,
            "setpoint_turns": self.turns,
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
            "addr": Address(response[0]),
            "code": Code(response[1]).name,
            "motor_real_time_speed": _signed_int(response[2:5]),
            "checksum": response[5],
        }

    @property
    def speed(self) -> int:
        return self.response_dict["motor_real_time_speed"]

    @property
    def value(self) -> int:
        return self.speed

    @property
    def parameter_dict(self) -> dict[str, int]:
        return {
            "motor_real_time_speed (RPM)": self.speed,
        }


@dataclass
class GetPosition(Command):
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
            "addr": Address(response[0]),
            "code": Code(response[1]).name,
            "motor_real_time_position": _signed_int(response[2:7]),
            "checksum": response[7],
        }

    @property
    def position(self) -> int:
        return self.response_dict["motor_real_time_position"]

    @property
    def value(self) -> int:
        return self.position

    @property
    def parameter_dict(self) -> dict[str, float]:
        return {
            "motor_real_time_position (deg)": self.deg,
            "motor_real_time_position (rad)": self.rad,
            "motor_real_time_turns": self.turns,
        }


@dataclass
class GetError(Position):
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
            "addr": Address(response[0]),
            "code": Code(response[1]).name,
            "motor_position_error": _signed_int(response[2:7]),
            "checksum": response[7],
        }

    @property
    def error(self) -> int:
        return self.response_dict["motor_position_error"]

    @property
    def value(self) -> int:
        return self.error

    @property
    def parameter_dict(self) -> dict[str, float]:
        return {
            "motor_position_error (deg)": self.deg,
            "motor_position_error (rad)": self.rad,
            "motor_position_error_turns": self.turns,
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

    @property
    def response_dict(self) -> dict[str, int]:
        response = self.response
        return {
            "addr": Address(response[0]),
            "code": Code(response[1]).name,
            "ready_status": StepperStatus(response[2]),
            "checksum": response[3],
        }

    @property
    def status(self) -> StepperStatus:
        return self.response_dict["ready_status"]

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
    def is_stall_protection(self) -> bool:
        return self.status.stall_protection


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
            "addr": Address(response[0]),
            "code": Code(response[1]),
            "byte_length": response[2],
            "param_count": response[3],
            "motor_type": MotorType(response[4]).name,
            "control_mode": ControlMode(response[5]).name,
            "communication_mode": CommunicationMode(response[6]).name,
            "enable_level": EnableLevel(response[7]).name,
            "direction": Direction(response[8]).name,
            "microsteps": Microstep(response[9]),
            "microstep_interp": MicrostepInterp(response[10]).name,
            "screen_off": ScreenOff(response[11]),
            "open_loop_current": _int(response[12:14]),
            "max_closed_loop_current": _int(response[14:16]),
            "max_voltage": _int(response[16:18]),
            "baud_rate": BaudRate(response[18]).name,
            "can_rate": CanRate(response[19]).name,
            "id_addr": Address(response[20]),
            "checksum_mode": ChecksumMode(response[21]).name,
            "response_mode": ResponseMode(response[22]).name,
            "stall_protect": StallProtect(response[23]),
            "stall_speed": _int(response[24:26]),
            "stall_current": _int(response[26:28]),
            "stall_time": _int(response[28:30]),
            "pos_window": _int(response[30:32]),  # 0.1XDeg
            "checksum": response[32],
        }

    @property
    def values(
        self,
    ) -> tuple[int, ...]:
        response = self.response_dict.copy()
        response.pop("checksum")
        response.pop("addr")
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
            "direction": Direction(response[8]).name,
            "microsteps": Microstep(response[9]),
            "microstep_interp": MicrostepInterp(response[10]).name,
            "screen_off": ScreenOff(response[11]).name,
            "open_loop_current (A)": _int(response[12:14]) * 0.001,
            "max_closed_loop_current (A)": _int(response[14:16]) * 0.001,
            "max_voltage (V)": _int(response[16:18]) * 0.001,
            "baud_rate": BaudRate(response[18]).name,
            "can_rate": CanRate(response[19]).name,
            "id_addr": Address(response[20]),
            "checksum_mode": ChecksumMode(response[21]).name,
            "response_mode": ResponseMode(response[22]).name,
            "stall_protect": StallProtect(response[23]).name,
            "stall_speed (RPM)": _int(response[24:26]),
            "stall_current (A)": _int(response[26:28]) * 0.001,
            "stall_time (s)": _int(response[28:30]) * 0.001,
            "pos_window (deg)": _int(response[30:32]) * 0.1,
        }


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
            "addr": Address(response[0]),
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
    def parameter_dict(self) -> dict[str, float | bool]:
        response = self.response_dict.copy()
        return {
            "bus_voltage (V)": _int(response[4:6]) * 0.001,
            "bus_phase_current (A)": _int(response[6:8]) * 0.001,
            "calibrated_encoder_value (deg)": _int(response[8:10]) * 0.1,
            "motor_target_position (deg)": _signed_int(response[10:15]) * 0.1,
            "motor_real_time_speed (RPM)": _signed_int(response[15:18]),
            "motor_real_time_position (deg)": _signed_int(response[18:23]) * 0.1,
            "motor_position_error (deg)": _signed_int(response[23:28]) * 0.1,
            "homing_encoder_ready": HomingStatus(response[28]).encoder_ready,
            "homing_encoder_calibrated": HomingStatus(response[28]).encoder_calibrated,
            "homing_is_homing": HomingStatus(response[28]).is_homing,
            "homing_failed": HomingStatus(response[28]).homing_failed,
            "stepper_enabled": StepperStatus(response[29]).enabled,
            "stepper_in_position": StepperStatus(response[29]).in_position,
            "stepper_stalled": StepperStatus(response[29]).stalled,
            "stepper_stall_protection": StepperStatus(response[29]).stall_protection,
        }
