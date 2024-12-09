"""Home commands for stepper motor."""

from stepper.commands.commands import (
    ReturnData,
    ReturnSuccess,
    TakeNoSetting,
    TakeStoreSetting,
    TakeSyncSetting,
    WithClassParams,
    WithEnumParams,
    WithNoParams,
)
from stepper.stepper_core.configs import (
    Code,
    HomingMode,
    Protocol,
)
from stepper.stepper_core.parameters import HomingParams, HomingStatus

__all__ = [
    "SetHome",
    "Home",
    "StopHome",
    "RetrieveHomeParam",
    "SetHomeParam",
    "GetHomeStatus",
]


class SetHome(WithNoParams, TakeStoreSetting, ReturnSuccess):
    """Set home command configuration."""

    _code: Code = Code.SET_HOME
    _protocol: Protocol = Protocol.SET_HOME


class Home(WithEnumParams, TakeSyncSetting, ReturnSuccess):
    """Home command configuration."""

    _code: Code = Code.HOME
    ParamsType = HomingMode


class StopHome(WithNoParams, TakeNoSetting, ReturnSuccess):
    """Stop homing command configuration."""

    _code: Code = Code.STOP_HOME
    _protocol: Protocol = Protocol.STOP_HOME


class RetrieveHomeParam(WithNoParams, TakeNoSetting, ReturnData):
    """Get home parameters command configuration."""

    _code: Code = Code.GET_HOME_PARAM
    _response_length: int = 18
    ReturnType = HomingParams


class SetHomeParam(WithClassParams, TakeStoreSetting, ReturnSuccess):
    """Set home parameters command configuration."""

    _code: Code = Code.SET_HOME_PARAM
    _protocol: Protocol = Protocol.SET_HOME_PARAM
    _command_lock: bool = True
    ParamsType = HomingParams


class GetHomeStatus(WithNoParams, TakeNoSetting, ReturnData):
    """Get home status command configuration."""

    _code: Code = Code.GET_HOME_STATUS
    _response_length = 4
    ReturnType = HomingStatus
