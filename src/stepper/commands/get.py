"""Get commands for stepper motor."""

from stepper.commands.commands import (
    Command,
    Protocol,
    ReturnData,
    TakeNoSetting,
    WithNoParams,
)
from stepper.stepper_core.configs import Code
from stepper.stepper_core.parameters import (
    BusVoltageParams,
    ConfigParams,
    EncoderParams,
    MotorRHParams,
    OpenLoopTargetPositionParams,
    PhaseCurrentParams,
    PIDParams,
    PositionErrorParams,
    PulseCountParams,
    RealTimePositionParams,
    RealTimeSpeedParams,
    StepperStatus,
    SystemParams,
    TargetPositionParams,
    VersionParams,
)

__all__ = [
    "GetVersion",
    "GetMotorRH",
    "GetPID",
    "GetBusVoltage",
    "GetPhaseCurrent",
    "GetEncoderValue",
    "GetPulseCount",
    "GetTargetPosition",
    "GetOpenLoopSetpoint",
    "GetRealTimeSpeed",
    "GetRealTimePosition",
    "GetPositionError",
    "GetStatus",
    "GetConfig",
    "GetSysStatus",
]


class GetCommand(WithNoParams, TakeNoSetting, ReturnData, Command):
    """Get command configuration."""


class GetVersion(GetCommand):
    """Get version of the device."""

    _code = Code.GET_VERSION
    _response_length = 5
    ReturnType = VersionParams


class GetMotorRH(GetCommand):
    """Get motor resistance and inductance."""

    _code = Code.GET_MOTOR_R_H
    _response_length = 7
    ReturnType = MotorRHParams


class GetPID(GetCommand):
    """Get PID parameters command configuration."""

    _code = Code.GET_PID
    _response_length = 15
    ReturnType = PIDParams


class GetBusVoltage(GetCommand):
    """Get the bus voltage."""

    _code = Code.GET_BUS_VOLTAGE
    _response_length = 5
    ReturnType = BusVoltageParams


class GetPhaseCurrent(GetCommand):
    """Get phase current."""

    _code = Code.GET_PHASE_CURRENT
    _response_length = 5
    ReturnType = PhaseCurrentParams


class GetEncoderValue(GetCommand):
    """Get encoder value."""

    _code = Code.GET_ENCODER_VALUE
    _response_length = 5
    ReturnType = EncoderParams


class GetPulseCount(GetCommand):
    """Get pulse count."""

    _code = Code.GET_PULSE_COUNT
    _response_length = 8
    ReturnType = PulseCountParams


class GetTargetPosition(GetCommand):
    """Get target position."""

    _code = Code.GET_TARGET
    _response_length = 8
    ReturnType = TargetPositionParams


class GetOpenLoopSetpoint(GetCommand):
    """Get open loop setpoint."""

    _code = Code.GET_OPEN_LOOP_SETPOINT
    _response_length = 8
    ReturnType = OpenLoopTargetPositionParams


class GetRealTimeSpeed(GetCommand):
    """Get real time speed in RPM."""

    _code = Code.GET_SPEED
    _response_length = 6
    ReturnType = RealTimeSpeedParams


class GetRealTimePosition(GetCommand):
    """Get real time position command configuration."""

    _code = Code.GET_POS
    _response_length = 8
    ReturnType = RealTimePositionParams


class GetPositionError(GetCommand):
    """Get error command configuration."""

    _code = Code.GET_ERROR
    _response_length = 8
    ReturnType = PositionErrorParams


class GetStatus(GetCommand):
    """Get status of the stepper motor."""

    _code = Code.GET_STATUS
    _response_length = 4
    ReturnType = StepperStatus


class GetConfig(GetCommand):
    """Get configuration."""

    _code = Code.GET_CONFIG
    _protocol = Protocol.GET_CONFIG
    _response_length = 33
    ReturnType = ConfigParams


class GetSysStatus(GetCommand):
    """Get system status command configuration."""

    _code = Code.GET_SYS_STATUS
    _protocol = Protocol.GET_SYS_STATUS
    _response_length = 31
    ReturnType = SystemParams
