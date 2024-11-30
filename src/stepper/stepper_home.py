from dataclasses import dataclass
from logging import getLogger
from .stepper_command import Command, _int
from .stepper_constants import (
    Code,
    Protocol,
    StoreFlag,
    SyncFlag,
    HomingMode,
    HomingDirection,
    HomingSpeed,
    HomingTimeout,
    CollisionDetectionSpeed,
    CollisionDetectionCurrent,
    CollisionDetectionTime,
    AutoHoming,
    HomingStatus,
    Address,
)

logger = getLogger(__name__)


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
            "code": Code(response[1]).name,
            "homing_mode": HomingMode(response[2]).name,
            "homing_direction": HomingDirection(response[3]).name,
            "homing_speed": HomingSpeed(_int(response[4:6])),
            "homing_timeout": HomingTimeout(_int(response[6:10])),
            "collision_detection_speed": CollisionDetectionSpeed(_int(response[10:12])),
            "collision_detection_current": CollisionDetectionCurrent(
                _int(response[12:14])
            ),
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
            "code": Code(response[1]).name,
            "homing_status": HomingStatus(response[2]).name,
            "checksum": response[3],
        }
