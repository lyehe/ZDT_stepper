"""
StepperLib - A robust library for stepper motor control
"""

import logging
import time

from .stepper_exceptions import (
    CommandError,
    ValidationError,
    CommunicationError,
    StatusError,
)
from .stepper_constants import (
    CMD_CODE,
    CHECKSUM_TYPE,
    PROTOCOL,
    HOMING_MODE,
    ENABLE_FLAG,
    DIRECTION,
    STORE_FLAG,
    SYNC_FLAG,
    ABSOLUTE_FLAG,
    Address,
    Speed,
    Acceleration,
    PulseCount,
    DEFAULT_ADDRESS,
    DEFAULT_CHECKSUM_TYPE,
    DEFAULT_SYNC_FLAG,
    DEFAULT_STORE_FLAG,
    DEFAULT_ABSOLUTE_FLAG,
    DEFAULT_HOMING_MODE,
    DEFAULT_DIRECTION,
)
from .stepper_dataclasses import CommandInput


def enable(
    device_number: Address = DEFAULT_ADDRESS,
    checksum_type: CHECKSUM_TYPE = DEFAULT_CHECKSUM_TYPE,
    sync: SYNC_FLAG = DEFAULT_SYNC_FLAG,
) -> CommandInput:
    data = PROTOCOL.ENABLE_CODE + ENABLE_FLAG.ENABLE + sync
    description = f"Enable motor {device_number}"
    return CommandInput(
        device_number=device_number,
        code=CMD_CODE.ENABLE.code,
        data=data,
        requires_enabled=False,
        description=description,
        checksum_type=checksum_type,
        expected_response_length=CMD_CODE.ENABLE.response_length,
    )


def disable(
    device_number: Address = DEFAULT_ADDRESS,
    checksum_type: CHECKSUM_TYPE = DEFAULT_CHECKSUM_TYPE,
    sync: SYNC_FLAG = DEFAULT_SYNC_FLAG,
) -> CommandInput:
    data = PROTOCOL.ENABLE_CODE + ENABLE_FLAG.DISABLE + sync
    description = f"Disable motor {device_number}"
    return CommandInput(
        device_number=device_number,
        code=CMD_CODE.ENABLE.code,
        data=data,
        requires_enabled=False,
        description=description,
        checksum_type=checksum_type,
        expected_response_length=CMD_CODE.ENABLE.response_length,
    )


def set_speed(
    device_number: Address = DEFAULT_ADDRESS,
    checksum_type: CHECKSUM_TYPE = DEFAULT_CHECKSUM_TYPE,
    direction: DIRECTION = DEFAULT_DIRECTION,
    speed: Speed = 0,
    acceleration: Acceleration = 0,
    sync: SYNC_FLAG = DEFAULT_SYNC_FLAG,
) -> CommandInput:
    data = direction + speed.to_bytes(2, "big") + acceleration.to_bytes(1, "big") + sync
    description = f"Set motor {device_number} speed to {speed} RPM (dir={direction}, accel={acceleration})"
    return CommandInput(
        device_number=device_number,
        code=CMD_CODE.SPEED.code,
        data=data,
        requires_enabled=True,
        description=description,
        checksum_type=checksum_type,
        expected_response_length=CMD_CODE.SPEED.response_length,
    )


def stop(
    device_number: Address = DEFAULT_ADDRESS,
    checksum_type: CHECKSUM_TYPE = DEFAULT_CHECKSUM_TYPE,
    sync: SYNC_FLAG = DEFAULT_SYNC_FLAG,
) -> CommandInput:
    """Special case of set_speed with speed=0"""
    return set_speed(
        device_number,
        checksum_type,
        direction=DEFAULT_DIRECTION,
        speed=0,
        acceleration=0,
        sync=sync,
    )


def set_position(
    device_number: Address = DEFAULT_ADDRESS,
    checksum_type: CHECKSUM_TYPE = DEFAULT_CHECKSUM_TYPE,
    direction: DIRECTION = DEFAULT_DIRECTION,
    speed: Speed = 0,
    acceleration: Acceleration = 0,
    pulse_count: PulseCount = 0,
    absolute: ABSOLUTE_FLAG = DEFAULT_ABSOLUTE_FLAG,
    sync: SYNC_FLAG = DEFAULT_SYNC_FLAG,
) -> CommandInput:
    data = (
        direction
        + speed.to_bytes(2, "big")
        + acceleration.to_bytes(1, "big")
        + pulse_count.to_bytes(4, "big")
        + absolute
        + sync
    )
    description = f"Set motor {device_number} position to {pulse_count} steps (dir={direction}, speed={speed}, accel={acceleration}, absolute={absolute})"
    return CommandInput(
        device_number=device_number,
        code=CMD_CODE.POSITION.code,
        data=data,
        requires_enabled=True,
        description=description,
        checksum_type=checksum_type,
        expected_response_length=CMD_CODE.POSITION.response_length,
    )


def emergency_stop(
    device_number: Address = DEFAULT_ADDRESS,
    checksum_type: CHECKSUM_TYPE = DEFAULT_CHECKSUM_TYPE,
    sync: SYNC_FLAG = DEFAULT_SYNC_FLAG,
) -> CommandInput:
    data = PROTOCOL.STOP_CODE + sync
    description = f"Emergency stop motor {device_number}"
    return CommandInput(
        device_number=device_number,
        code=CMD_CODE.EMERGENCY_STOP.code,
        data=data,
        requires_enabled=False,
        description=description,
        checksum_type=checksum_type,
        expected_response_length=CMD_CODE.EMERGENCY_STOP.response_length,
    )


def sync_move(
    device_number: Address = DEFAULT_ADDRESS,
    checksum_type: CHECKSUM_TYPE = DEFAULT_CHECKSUM_TYPE,
) -> CommandInput:
    data = PROTOCOL.SYNC_MOVE_CODE
    description = "Sync move all motors"
    return CommandInput(
        device_number=device_number,
        code=CMD_CODE.SYNC_MOVE.code,
        data=data,
        requires_enabled=True,
        description=description,
        checksum_type=checksum_type,
        expected_response_length=CMD_CODE.SYNC_MOVE.response_length,
    )


def set_home(
    device_number: Address = DEFAULT_ADDRESS,
    checksum_type: CHECKSUM_TYPE = DEFAULT_CHECKSUM_TYPE,
    store: STORE_FLAG = DEFAULT_STORE_FLAG,
) -> CommandInput:
    data = PROTOCOL.SET_HOME_CODE + store
    description = f"Set home position for motor {device_number}"
    return CommandInput(
        device_number=device_number,
        code=CMD_CODE.SET_HOME.code,
        data=data,
        requires_enabled=False,
        description=description,
        checksum_type=checksum_type,
        expected_response_length=CMD_CODE.SET_HOME.response_length,
    )


def homing(
    device_number: Address = DEFAULT_ADDRESS,
    checksum_type: CHECKSUM_TYPE = DEFAULT_CHECKSUM_TYPE,
    homing_mode: HOMING_MODE = DEFAULT_HOMING_MODE,
    sync: SYNC_FLAG = DEFAULT_SYNC_FLAG,
) -> CommandInput:
    data = homing_mode + sync
    description = f"Homing motor {device_number}"
    return CommandInput(
        device_number=device_number,
        code=CMD_CODE.HOMING.code,
        data=data,
        requires_enabled=True,
        description=description,
        checksum_type=checksum_type,
        expected_response_length=CMD_CODE.HOMING.response_length,
    )


def abort_home(
    device_number: Address = DEFAULT_ADDRESS,
    checksum_type: CHECKSUM_TYPE = DEFAULT_CHECKSUM_TYPE,
) -> CommandInput:
    data = PROTOCOL.ABORT_HOME_CODE
    description = f"Abort homing for motor {device_number}"
    return CommandInput(
        device_number=device_number,
        code=CMD_CODE.ABORT_HOME.code,
        data=data,
        requires_enabled=False,
        description=description,
        checksum_type=checksum_type,
        expected_response_length=CMD_CODE.ABORT_HOME.response_length,
    )


def read_homing_status(
    device_number: Address = DEFAULT_ADDRESS,
    checksum_type: CHECKSUM_TYPE = DEFAULT_CHECKSUM_TYPE,
) -> CommandInput:
    data = bytes([])
    description = f"Read homing status for motor {device_number}"
    return CommandInput(
        device_number=device_number,
        code=CMD_CODE.READ_HOMING_STATUS.code,
        data=data,
        requires_enabled=False,
        description=description,
        checksum_type=checksum_type,
        expected_response_length=CMD_CODE.READ_HOMING_STATUS.response_length,
    )

