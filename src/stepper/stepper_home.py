"""Home commands for stepper motor."""

from logging import getLogger

from .stepper_command import (
    ReturnData,
    ReturnSuccess,
    TakeNoSetting,
    TakeStoreSetting,
    TakeSyncSetting,
    WithClassParams,
    WithEnumParams,
    WithNoParams,
    to_int,
)
from .stepper_constants import (
    AutoHoming,
    Code,
    CollisionDetectionCurrent,
    CollisionDetectionSpeed,
    CollisionDetectionTime,
    HomingDirection,
    HomingMode,
    HomingSpeed,
    HomingTimeout,
    Protocol,
)
from .stepper_parameters import HomingParams, HomingStatus

logger = getLogger(__name__)


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

    def _unpack_data(self, data: bytes) -> HomingParams:
        """Return home parameters as a dictionary."""
        return HomingParams(
            homing_mode=HomingMode(data[0]),
            homing_direction=HomingDirection(data[1]),
            homing_speed=HomingSpeed(to_int(data[2:4])),
            homing_timeout=HomingTimeout(to_int(data[4:8])),
            collision_detection_speed=CollisionDetectionSpeed(to_int(data[8:10])),
            collision_detection_current=CollisionDetectionCurrent(to_int(data[10:12])),
            collision_detection_time=CollisionDetectionTime(to_int(data[12:14])),
            auto_home=AutoHoming(data[14]),
        )


class SetHomeParam(WithClassParams, TakeStoreSetting, ReturnSuccess):
    """Set home parameters command configuration."""

    _code: Code = Code.SET_HOME_PARAM
    _protocol: Protocol = Protocol.SET_HOME_PARAM
    ParamsType = HomingParams


class GetHomeStatus(WithNoParams, TakeNoSetting, ReturnData):
    """Get home status command configuration."""

    _code: Code = Code.GET_HOME_STATUS
    _response_length = 4
    ReturnType = HomingStatus
