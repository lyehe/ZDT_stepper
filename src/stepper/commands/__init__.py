"""Command classes for stepper motor control."""

from .get import (
    GetBusVoltage,
    GetConfig,
    GetEncoderValue,
    GetError,
    GetMotorRH,
    GetOpenLoopSetpoint,
    GetPhaseCurrent,
    GetPID,
    GetPulseCount,
    GetRealTimePosition,
    GetRealTimeSpeed,
    GetStatus,
    GetSysStatus,
    GetTargetPosition,
    GetVersion,
)
from .home import (
    GetHomeStatus,
    Home,
    RetrieveHomeParam,
    SetHome,
    SetHomeParam,
    StopHome,
)
from .move import (
    Disable,
    Enable,
    EStop,
    Jog,
    Move,
    SyncMove,
)
from .set import (
    SetConfig,
    SetID,
    SetLoopMode,
    SetMicrostep,
    SetOpenLoopCurrent,
    SetPID,
    SetReduction,
    SetStartSpeed,
)
from .system import (
    CalibrateEncoder,
    ClearStall,
    FactoryReset,
    ZeroAllPositions,
)

__all__ = [
    # Get commands
    "GetBusVoltage",
    "GetConfig",
    "GetEncoderValue",
    "GetError",
    "GetHomeStatus",
    "GetMotorRH",
    "GetOpenLoopSetpoint",
    "GetPhaseCurrent",
    "GetPID",
    "GetPulseCount",
    "GetRealTimePosition",
    "GetRealTimeSpeed",
    "GetStatus",
    "GetSysStatus",
    "GetTargetPosition",
    "GetVersion",
    # Home commands
    "GetHomeStatus",
    "Home",
    "RetrieveHomeParam",
    "SetHome",
    "SetHomeParam",
    "StopHome",
    # Move commands
    "Disable",
    "Enable",
    "EStop",
    "Jog",
    "Move",
    "SyncMove",
    # Set commands
    "SetConfig",
    "SetID",
    "SetLoopMode",
    "SetMicrostep",
    "SetOpenLoopCurrent",
    "SetPID",
    "SetReduction",
    "SetStartSpeed",
    # System commands
    "CalibrateEncoder",
    "ClearStall",
    "FactoryReset",
    "ZeroAllPositions",
]
