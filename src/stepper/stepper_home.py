from dataclasses import asdict, dataclass
from logging import getLogger

from .stepper_command import BroadcastCommand, Command, _int
from .stepper_constants import (
    Address,
    AutoHoming,
    Code,
    CollisionDetectionCurrent,
    CollisionDetectionSpeed,
    CollisionDetectionTime,
    HomingDirection,
    HomingMode,
    HomingSpeed,
    HomingStatus,
    HomingTimeout,
    Protocol,
    StoreFlag,
    SyncFlag,
)

logger = getLogger(__name__)


@dataclass
class SetHome(Command):
    """Set home command configuration

    :param store: Store flag
    """

    store: StoreFlag = StoreFlag.default

    @property
    def _code(self) -> Code:
        return Code.SET_HOME

    @property
    def _input(self) -> bytes:
        return bytes([self.addr, self._code, Protocol.SET_HOME, self.store])


@dataclass
class SetHomeAll(SetHome, BroadcastCommand):
    """Set home all command configuration"""


@dataclass
class Home(Command):
    """Home command configuration

    :param homing_mode: Homing mode
    :param sync: Sync flag
    """

    homing_mode: HomingMode = HomingMode.default
    sync: SyncFlag = SyncFlag.default

    @property
    def _code(self) -> Code:
        return Code.HOME

    @property
    def _input(self) -> bytes:
        return bytes([self.addr, self._code, self.homing_mode, self.sync])


@dataclass
class HomeAll(Home, BroadcastCommand):
    """Home all command configuration"""


@dataclass
class StopHome(Command):
    """Stop homing command configuration"""

    @property
    def _code(self) -> Code:
        return Code.STOP_HOME

    @property
    def _input(self) -> bytes:
        return bytes([self.addr, self._code, Protocol.STOP_HOME])


@dataclass
class GetHomeParam(Command):
    """Get home parameters command configuration"""

    @property
    def _code(self) -> Code:
        return Code.GET_HOME_PARAM

    @property
    def _input(self) -> bytes:
        return bytes([self.addr, self._code])

    @property
    def response_dict(self) -> dict[str, int]:
        response = self.response
        return {
            "addr": Address(response[0]),
            "code": Code(response[1]).name,
            "homing_mode": HomingMode(response[2]).name,
            "homing_direction": HomingDirection(response[3]).name,
            "homing_speed": HomingSpeed(_int(response[4:6])),
            "homing_timeout": HomingTimeout(_int(response[6:10])),
            "collision_detection_speed": CollisionDetectionSpeed(_int(response[10:12])),
            "collision_detection_current": CollisionDetectionCurrent(_int(response[12:14])),
            "collision_detection_time": CollisionDetectionTime(_int(response[14:16])),
            "auto_home": AutoHoming(response[16]).name,
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

    store: StoreFlag = StoreFlag.default
    homing_mode: HomingMode = HomingMode.default
    homing_direction: HomingDirection = HomingDirection.default
    homing_speed: HomingSpeed = HomingSpeed.default
    homing_timeout: HomingTimeout = HomingTimeout.default
    collision_detection_speed: CollisionDetectionSpeed = CollisionDetectionSpeed.default
    collision_detection_current: CollisionDetectionCurrent = CollisionDetectionCurrent.default
    collision_detection_time: CollisionDetectionTime = CollisionDetectionTime.default
    auto_home: AutoHoming = AutoHoming.default

    @property
    def _code(self) -> Code:
        return Code.SET_HOME_PARAM

    @property
    def _input(self) -> bytes:
        return bytes(
            [
                self.addr,
                self._code,
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
    def _code(self) -> Code:
        return Code.GET_HOME_STATUS

    @property
    def _input(self) -> bytes:
        return bytes([self.addr, self._code])

    @property
    def response_dict(self) -> dict[str, int]:
        response = self.response
        return {
            "addr": Address(response[0]),
            "code": Code(response[1]).name,
            "homing_status": HomingStatus(response[2]).name,
            "checksum": response[3],
        }

    @property
    def homing_status(self) -> HomingStatus:
        return HomingStatus(self.response_dict["homing_status"])

    @property
    def parameter_dict(self) -> dict[str, bool]:
        return asdict(self.homing_status)
