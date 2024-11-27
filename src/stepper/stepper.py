"""
StepperLib - A robust library for stepper motor control
"""

from dataclasses import dataclass
from enum import Enum, IntEnum, auto
from typing import TypeAlias, Literal, Final
import logging
import time
from serial import Serial
from pathlib import Path

# Type definitions
Address: TypeAlias = int
CommandCode: TypeAlias = int
Status: TypeAlias = int
Direction = Literal[0, 1]  # 0=CW, 1=CCW
StoreFlag = Literal[0, 1]  # 0=temporary, 1=permanent
SyncFlag = Literal[0, 1]  # 0=no sync, 1=sync


# Command codes
class CMD(IntEnum):
    """Command codes for stepper motor protocol"""

    # Control Commands
    ENABLE = 0xF3
    SPEED = 0xF6
    POSITION = 0xFD
    EMERGENCY_STOP = 0xFE
    SYNC_MOVE = 0xFF

    # Homing Commands
    SET_ZERO = 0x93
    TRIGGER_HOME = 0x9A
    ABORT_HOME = 0x9C

    # Action Commands
    CALIBRATE = 0x06
    CLEAR_POS = 0x0A
    CLEAR_STALL = 0x0E
    FACTORY_RESET = 0x0F

    # Read Commands
    READ_VERSION = 0x1F
    READ_PHASE_PARAMS = 0x20
    READ_PID = 0x21
    READ_BUS_VOLTAGE = 0x24
    READ_PHASE_CURRENT = 0x27
    READ_CALIBRATED_ENCODER = 0x31
    READ_INPUT_PULSE_COUNT = 0x32
    READ_TARGET = 0x33  # Target position
    READ_REALTIME_TARGET = 0x34
    READ_REALTIME_SPEED = 0x35
    READ_POSITION = 0x36
    READ_POSITION_ERROR = 0x37
    READ_STATUS = 0x3A
    READ_HOMING_STATUS = 0x3B
    READ_DRIVE_CONFIG = 0x42
    READ_SYSTEM_STATUS = 0x43

    # Configuration Commands
    MODIFY_SUBDIVISION = 0x84
    MODIFY_ID_ADDRESS = 0xAE
    SWITCH_LOOP_MODE = 0x46
    MODIFY_OPEN_LOOP_CURRENT = 0x44
    MODIFY_PID_PARAMS = 0x4A
    STORE_SPEED_PARAMS = 0xF7
    MODIFY_SPEED_SCALE = 0x4F


# Protocol constants
SYNC_BYTE: Final[int] = 0xAB
CHECKSUM_BYTE: Final[int] = 0x6B
SUCCESS_CODE: Final[int] = 0x02
ERROR_CODE: Final[int] = 0xEE
CONDITION_ERROR: Final[int] = 0xE2


class MotorError(Exception):
    """Base exception for motor errors"""


class CommandError(MotorError):
    """Error executing a command"""


class ValidationError(MotorError):
    """Error validating command parameters"""


class CommunicationError(MotorError):
    """Error communicating with the motor"""


class StatusError(MotorError):
    """Error with motor status"""


@dataclass(frozen=True)
class CommandResult:
    """Result of a motor command execution"""

    success: bool
    code: int
    data: bytes | None = None

    @property
    def is_error(self) -> bool:
        return not self.success

    @property
    def is_condition_error(self) -> bool:
        return self.code == CONDITION_ERROR


@dataclass
class MotorStatus:
    """Motor status information"""

    enabled: bool = False
    position_reached: bool = False
    stalled: bool = False
    protection_triggered: bool = False

    @classmethod
    def from_byte(cls, status: int) -> "MotorStatus":
        return cls(
            enabled=bool(status & 0x01),
            position_reached=bool(status & 0x02),
            stalled=bool(status & 0x04),
            protection_triggered=bool(status & 0x08),
        )


@dataclass
class Position:
    """Motor position information"""

    steps: int
    revolutions: int
    degrees: float

    @classmethod
    def from_bytes(cls, data: bytes) -> "Position":
        if len(data) < 4:
            raise ValueError("Position data must be at least 4 bytes")

        sign = -1 if data[0] else 1
        steps = sign * int.from_bytes(data[1:], "big")
        revolutions = steps // 65536
        degrees = (abs(steps) % 65536) * 360 / 65536
        return cls(steps=steps, revolutions=revolutions, degrees=degrees)

    def __str__(self) -> str:
        return f"Position(steps={self.steps}, revolutions={self.revolutions}, degrees={self.degrees:.1f})"


class Command:
    """Base class for motor commands"""

    def __init__(
        self,
        code: CommandCode,
        *,
        data: bytes = b"",
        requires_enabled: bool = True,
        timeout: float = 1.0,
        retries: int = 3,
        description: str = "",
    ):
        self.code = code
        self.data = data
        self.requires_enabled = requires_enabled
        self.timeout = timeout
        self.retries = retries
        self.description = description or CMD(code).name

    def build(self, address: Address) -> bytes:
        """Build complete command bytes"""
        cmd = bytes([address, self.code]) + self.data
        return cmd + bytes([CHECKSUM_BYTE])

    def parse_response(self, response: bytes) -> CommandResult:
        """Parse command response"""
        if len(response) < 4:
            raise CommandError(f"Response too short: {len(response)} bytes")

        # For status commands, always treat as success if we got a response
        if self.code in (CMD.READ_STATUS, CMD.READ_HOMING_STATUS):
            return CommandResult(
                success=True,
                code=response[2],  # The status byte
                data=bytes([response[2]]),  # Include status byte as data
            )

        # Rest of the method remains the same
        is_read_command = (self.code & 0xF0) <= 0x40
        if is_read_command:
            success = response[2] != ERROR_CODE and response[2] != CONDITION_ERROR
        else:
            success = response[2] == SUCCESS_CODE

        code = response[2]
        data = response[3:-1] if len(response) > 4 else None

        return CommandResult(success=success, code=code, data=data)


@dataclass
class EncoderCalibrationStatus:
    """Encoder calibration status information"""

    is_calibrated: bool = False
    calibration_table_ready: bool = False
    zero_position_set: bool = False

    @classmethod
    def from_byte(cls, status: int) -> "EncoderCalibrationStatus":
        return cls(
            is_calibrated=bool(status & 0x01),
            calibration_table_ready=bool(status & 0x02),
            zero_position_set=bool(status & 0x04),
        )


class StepperMotor:
    """Main stepper motor control class"""

    def __init__(
        self,
        port: str | Path,
        address: Address = 0x01,
        baudrate: int = 115200,
        timeout: float = 1.0,
    ):
        if not 1 <= address <= 15:
            raise ValidationError("Address must be between 1 and 15")
        self.address = address
        self.timeout = timeout
        self._enabled = None
        self._serial = Serial(
            port=str(port), baudrate=baudrate, timeout=timeout, write_timeout=timeout
        )
        self._logger = logging.getLogger("StepperMotor")

    @property
    def is_enabled(self) -> bool:
        return self._enabled or self.get_status().enabled

    def _execute(self, command: Command) -> CommandResult:
        """Execute a command and return the result"""
        if (
            self._enabled is not None
            and command.requires_enabled
            and not self.is_enabled
        ):
            raise StatusError("Motor must be enabled")

        cmd_bytes = command.build(self.address)
        retries = command.retries

        # Log the command with semantic meaning
        self._logger.debug(
            f"Command: {command.description}\n" f"TX: {cmd_bytes.hex(' ').upper()}"
        )

        while retries >= 0:
            try:
                self._serial.write(cmd_bytes)
                response = self._serial.read_until(bytes([CHECKSUM_BYTE]))
                if not response:
                    raise CommunicationError("No response received")

                # Log the response with command context
                self._logger.debug(
                    f"Response to {command.description}:\n"
                    f"RX: {response.hex(' ').upper()}"
                )

                return command.parse_response(response)
            except Exception as e:
                if retries == 0:
                    raise
                retries -= 1
                self._logger.warning(
                    f"Command {command.description} failed, retrying: {e}"
                )
                time.sleep(0.1)

    def enable(self, store: StoreFlag = 0, sync: SyncFlag = 0) -> None:
        """Enable the motor"""
        cmd = Command(
            CMD.ENABLE,
            data=bytes([SYNC_BYTE, 1, sync]),
            requires_enabled=False,
            description=f"Enable motor (store={store}, sync={sync})",
        )
        result = self._execute(cmd)
        if result.success:
            self._enabled = True
        elif result.is_condition_error:
            raise StatusError("Motor stalled")
        else:
            raise CommandError("Failed to enable motor")

    def disable(self) -> None:
        """Disable the motor"""
        cmd = Command(
            CMD.ENABLE,
            data=bytes([SYNC_BYTE, 0, 0]),
            requires_enabled=False,
            description="Disable motor",
        )
        result = self._execute(cmd)
        if result.success:
            self._enabled = False

    def emergency_stop(self) -> None:
        """Emergency stop the motor"""
        cmd = Command(
            CMD.EMERGENCY_STOP,
            data=bytes([0x98, 0]),
            requires_enabled=False,
            description="Emergency stop",
        )
        self._execute(cmd)
        self._enabled = False

    def set_speed(
        self,
        rpm: int,
        direction: Direction = 1,
        acceleration: int = 0,
        sync: SyncFlag = 0,
    ) -> None:
        """Set motor speed"""
        if not 0 <= rpm <= 65535:
            raise ValidationError("Speed must be 0-65535 RPM")
        if not 0 <= acceleration <= 255:
            raise ValidationError("Acceleration must be 0-255")

        cmd = Command(
            CMD.SPEED,
            data=bytes([direction])
            + rpm.to_bytes(2, "big")
            + bytes([acceleration, sync]),
            description=f"Set speed to {rpm} RPM (dir={direction}, accel={acceleration})",
        )
        result = self._execute(cmd)
        if result.is_condition_error:
            raise StatusError("Motor stalled or disabled")

    def set_position(
        self,
        steps: int,
        speed: int,
        direction: Direction = 1,
        acceleration: int = 0,
        relative: bool = True,
        sync: SyncFlag = 0,
    ) -> None:
        """Move motor to position"""
        if not 0 <= speed <= 65535:
            raise ValidationError("Speed must be 0-65535 RPM")
        if not 0 <= acceleration <= 255:
            raise ValidationError("Acceleration must be 0-255")

        cmd = Command(
            CMD.POSITION,
            data=bytes([direction])
            + speed.to_bytes(2, "big")
            + bytes([acceleration])
            + steps.to_bytes(4, "big")
            + bytes([0 if relative else 1, sync]),
            description=(
                f"Move {'relative' if relative else 'absolute'} "
                f"{steps} steps at {speed} RPM "
                f"(dir={direction}, accel={acceleration})"
            ),
        )
        result = self._execute(cmd)
        if result.is_condition_error:
            raise StatusError("Motor stalled or disabled")

    def get_position(self) -> Position:
        """Get current motor position"""
        cmd = Command(CMD.READ_POSITION, description="Read current position")
        result = self._execute(cmd)
        if result.success and result.data:
            return Position.from_bytes(result.data)

        raise CommandError("Failed to read position")

    def get_status(self) -> MotorStatus:
        """Get motor status"""
        cmd = Command(CMD.READ_STATUS, description="Read motor status")
        result = self._execute(cmd)
        if result.success and result.data:
            return MotorStatus.from_byte(result.data[0])
        raise CommandError("Failed to read status")

    def set_zero(self, store: StoreFlag = 1) -> None:
        """Set current position as zero"""
        cmd = Command(
            CMD.SET_ZERO,
            data=bytes([0x88, store]),
            description=f"Set zero position (store={store})",
        )
        self._execute(cmd)

    def home(self, mode: int = 0, sync: SyncFlag = 0) -> None:
        """Home the motor

        Args:
            mode: Homing mode
                0: Single-turn nearest
                1: Single-turn directional
                2: Multi-turn collision
                3: Multi-turn limit switch
            sync: Sync flag (0=no sync, 1=sync)
        """
        if not 0 <= mode <= 3:
            raise ValidationError("Invalid homing mode")

        # Check motor status before homing
        status = self.get_status()
        if not status.enabled:
            raise StatusError("Motor must be enabled before homing")
        if status.stalled:
            raise StatusError("Motor is stalled - clear stall protection first")

        mode_desc = {
            0: "single-turn nearest",
            1: "single-turn directional",
            2: "multi-turn collision",
            3: "multi-turn limit switch",
        }

        cmd = Command(
            CMD.TRIGGER_HOME,
            data=bytes([mode, sync]),
            description=f"Home motor ({mode_desc[mode]})",
            timeout=10.0,  # Increase timeout for homing operation
        )
        result = self._execute(cmd)
        if result.is_condition_error:
            # Check status again to provide more specific error
            status = self.get_status()
            if status.stalled:
                raise StatusError("Motor stalled during homing")
            elif not status.enabled:
                raise StatusError("Motor disabled during homing")
            else:
                raise StatusError("Homing failed - check encoder calibration")

    def abort_home(self) -> None:
        """Abort homing operation"""
        cmd = Command(
            CMD.ABORT_HOME, data=bytes([0x48]), description="Abort homing operation"
        )
        result = self._execute(cmd)
        if result.is_condition_error:
            raise StatusError("No homing in progress")

    def clear_stall(self) -> None:
        """Clear stall protection"""
        cmd = Command(
            CMD.CLEAR_STALL, data=bytes([0x52]), description="Clear stall protection"
        )
        result = self._execute(cmd)
        if result.is_condition_error:
            raise StatusError("No stall protection triggered")

    def factory_reset(self) -> None:
        """Reset to factory settings"""
        cmd = Command(
            CMD.FACTORY_RESET,
            data=bytes([0x5F]),
            requires_enabled=False,
            description="Factory reset",
        )
        self._execute(cmd)
        self._enabled = False

    def close(self) -> None:
        """Close serial connection"""
        self._serial.close()

    def __enter__(self) -> "StepperMotor":
        return self

    def __exit__(self, *args) -> None:
        self.close()

    def get_realtime_target(self) -> Position:
        """Get real-time target position"""
        cmd = Command(
            CMD.READ_REALTIME_TARGET, description="Read real-time target position"
        )
        result = self._execute(cmd)
        if result.success and result.data:
            return Position.from_bytes(result.data)
        raise CommandError("Failed to read target position")

    def get_realtime_speed(self) -> int:
        """Get real-time speed in RPM"""
        cmd = Command(CMD.READ_REALTIME_SPEED, description="Read real-time speed")
        result = self._execute(cmd)
        if result.success and result.data:
            sign = -1 if result.data[0] else 1
            speed = int.from_bytes(result.data[1:3], "big")
            return sign * speed
        raise CommandError("Failed to read speed")

    def get_position_error(self) -> int:
        """Get position error in steps"""
        cmd = Command(CMD.READ_POSITION_ERROR, description="Read position error")
        result = self._execute(cmd)
        if result.success and result.data:
            sign = -1 if result.data[0] else 1
            error = int.from_bytes(result.data[1:5], "big")
            return sign * error
        raise CommandError("Failed to read position error")

    def get_homing_status(self) -> dict[str, bool]:
        """Get homing status flags

        Returns:
            Dictionary with status flags:
            - encoder_ready: Encoder is ready
            - calibration_ready: Calibration table is ready
            - homing_in_progress: Homing operation in progress
            - homing_failed: Homing operation failed
        """
        cmd = Command(
            CMD.READ_HOMING_STATUS,
            requires_enabled=False,  # Status can be read when disabled
            description="Read homing status",
        )
        result = self._execute(cmd)
        # The response format is: [Address] [CMD] [Status] [Checksum]
        # We're getting: 01 3B 03 6B which is a valid response
        if result.success:  # Remove the data check since we always get data
            status = result.data[0]
            return {
                "encoder_ready": bool(status & 0x01),
                "calibration_ready": bool(status & 0x02),
                "homing_in_progress": bool(status & 0x04),
                "homing_failed": bool(status & 0x08),
            }

        # Only raise error if command actually failed
        self._logger.error(
            f"Failed to read homing status. Result: success={result.success}, code=0x{result.code:02X}"
        )
        raise CommandError("Failed to read homing status")

    def modify_subdivision(self, subdivision: int, store: StoreFlag = 1) -> None:
        """Modify motor subdivision setting

        Args:
            subdivision: Subdivision value (0=256, 1-255)
            store: Store to memory flag
        """
        if not 0 <= subdivision <= 255:
            raise ValidationError("Subdivision must be 0-255")

        cmd = Command(
            CMD.MODIFY_SUBDIVISION,
            data=bytes([0x8A, store, subdivision]),
            description=f"Set subdivision to {subdivision}",
        )
        self._execute(cmd)

    def modify_address(self, new_address: int, store: StoreFlag = 1) -> None:
        """Modify motor address

        Args:
            new_address: New address (1-15)
            store: Store to memory flag
        """
        if not 1 <= new_address <= 15:
            raise ValidationError("Address must be 1-15")

        cmd = Command(
            CMD.MODIFY_ID_ADDRESS,
            data=bytes([0x4B, store, new_address]),
            description=f"Change address to {new_address}",
        )
        result = self._execute(cmd)
        if result.success:
            self.address = new_address

    def set_control_mode(self, open_loop: bool, store: StoreFlag = 1) -> None:
        """Set motor control mode

        Args:
            open_loop: True for open loop, False for closed loop
            store: Store to memory flag
        """
        mode = 1 if open_loop else 2
        cmd = Command(
            CMD.SWITCH_LOOP_MODE,
            data=bytes([0x69, store, mode]),
            description=f"Set {'open' if open_loop else 'closed'} loop mode",
        )
        self._execute(cmd)

    def set_open_loop_current(self, current_ma: int, store: StoreFlag = 1) -> None:
        """Set open loop current in mA

        Args:
            current_ma: Current in milliamps
            store: Store to memory flag
        """
        if not 0 <= current_ma <= 65535:
            raise ValidationError("Current must be 0-65535 mA")

        cmd = Command(
            CMD.MODIFY_OPEN_LOOP_CURRENT,
            data=bytes([0x33, store]) + current_ma.to_bytes(2, "big"),
            description=f"Set open loop current to {current_ma}mA",
        )
        self._execute(cmd)

    def set_pid_parameters(
        self, kp: int, ki: int, kd: int, store: StoreFlag = 1
    ) -> None:
        """Set position PID parameters

        Args:
            kp: Proportional gain (32-bit)
            ki: Integral gain (32-bit)
            kd: Derivative gain (32-bit)
            store: Store to memory flag
        """
        cmd = Command(
            CMD.MODIFY_PID_PARAMS,
            data=bytes([0xC3, store])
            + kp.to_bytes(4, "big")
            + ki.to_bytes(4, "big")
            + kd.to_bytes(4, "big"),
            description=f"Set PID parameters (Kp={kp}, Ki={ki}, Kd={kd})",
        )
        self._execute(cmd)

    def store_speed_params(
        self,
        rpm: int,
        direction: Direction = 1,
        acceleration: int = 0,
        enable: bool = True,
    ) -> None:
        """Store default speed mode parameters

        Args:
            rpm: Speed in RPM
            direction: Rotation direction
            acceleration: Acceleration (0-255)
            enable: Enable motor after power-up
        """
        if not 0 <= rpm <= 65535:
            raise ValidationError("Speed must be 0-65535 RPM")
        if not 0 <= acceleration <= 255:
            raise ValidationError("Acceleration must be 0-255")

        cmd = Command(
            CMD.STORE_SPEED_PARAMS,
            data=bytes([0x1C, 1, direction])
            + rpm.to_bytes(2, "big")
            + bytes([acceleration, 1 if enable else 0]),
            description=f"Store speed parameters ({rpm} RPM)",
        )
        self._execute(cmd)

    def set_speed_scale(self, divide_by_ten: bool, store: StoreFlag = 1) -> None:
        """Set speed input scaling

        Args:
            divide_by_ten: If True, input speeds will be divided by 10
            store: Store to memory flag
        """
        cmd = Command(
            CMD.MODIFY_SPEED_SCALE,
            data=bytes([0x71, store, 1 if divide_by_ten else 0]),
            description=f"Set speed scale ({'÷10' if divide_by_ten else 'normal'})",
        )
        self._execute(cmd)

    def get_phase_parameters(self) -> dict[str, float]:
        """Get motor phase parameters

        Returns:
            Dictionary with resistance (mΩ) and inductance (μH)
        """
        cmd = Command(CMD.READ_PHASE_PARAMS, description="Read phase parameters")
        result = self._execute(cmd)
        if result.success and result.data:
            resistance = int.from_bytes(result.data[0:2], "big")  # mΩ
            inductance = int.from_bytes(result.data[2:4], "big")  # μH
            return {"resistance": resistance, "inductance": inductance}
        raise CommandError("Failed to read phase parameters")

    def get_bus_voltage(self) -> float:
        """Get bus voltage in volts"""
        cmd = Command(CMD.READ_BUS_VOLTAGE, description="Read bus voltage")
        result = self._execute(cmd)
        if result.success and result.data:
            voltage_mv = int.from_bytes(result.data, "big")
            return voltage_mv / 1000.0  # Convert mV to V
        raise CommandError("Failed to read bus voltage")

    def get_phase_current(self) -> float:
        """Get phase current in amps"""
        cmd = Command(CMD.READ_PHASE_CURRENT, description="Read phase current")
        result = self._execute(cmd)
        if result.success and result.data:
            current_ma = int.from_bytes(result.data, "big")
            return current_ma / 1000.0  # Convert mA to A
        raise CommandError("Failed to read phase current")

    def get_calibrated_encoder(self) -> int:
        """Get calibrated encoder value (0-65535 per revolution)"""
        cmd = Command(
            CMD.READ_CALIBRATED_ENCODER, description="Read calibrated encoder value"
        )
        result = self._execute(cmd)
        if result.success and result.data:
            return int.from_bytes(result.data, "big")
        raise CommandError("Failed to read encoder value")

    def get_input_pulse_count(self) -> int:
        """Get input pulse count with direction"""
        cmd = Command(CMD.READ_INPUT_PULSE_COUNT, description="Read input pulse count")
        result = self._execute(cmd)
        if result.success and result.data:
            sign = -1 if result.data[0] else 1
            count = int.from_bytes(result.data[1:5], "big")
            return sign * count
        raise CommandError("Failed to read pulse count")

    def get_drive_config(self) -> dict:
        """Get drive configuration parameters

        Returns:
            Dictionary containing all drive configuration parameters:
            - byte_count: Number of data bytes (33)
            - param_count: Number of parameters (21)
            - motor_type: '1.8°' or '0.9°' motor
            - pulse_control_mode: Control mode value
            - communication_mode: Communication interface type
            - en_pin_level: Enable pin level setting
            - dir_pin_direction: 'CW' or 'CCW'
            - subdivision: Subdivision setting (256 if 0)
            - subdivision_interpolation: True/False
            - auto_screen_off: True/False
            - open_loop_current: Current in mA
            - max_closed_loop_current: Current in mA
            - max_output_voltage: Voltage in mV
            - uart_baud_rate: Baud rate in bps
            - can_rate: CAN bus rate in bps
            - device_id: Device ID (1-255)
            - checksum: Communication checksum
            - command_response_mode: 'Full' or 'Acknowledge'
            - stall_protection: True/False
            - stall_speed_threshold: Speed in RPM
            - stall_current_threshold: Current in mA
            - stall_detection_time: Time in ms
            - position_arrival_window: Window in degrees
        """
        cmd = Command(
            CMD.READ_DRIVE_CONFIG,
            data=bytes([0x6C]),
            description="Read drive configuration",
            requires_enabled=False,  # Configuration can be read when disabled
        )
        result = self._execute(cmd)

        # Add debug logging
        self._logger.debug(
            f"Drive config response: success={result.success}, code=0x{result.code:02X}, data_length={len(result.data) if result.data else 0}"
        )

        # Check response validity
        if not result.success:
            raise CommandError(
                f"Drive configuration command failed with code: 0x{result.code:02X}"
            )

        if not result.data:
            raise CommandError("No data received from drive configuration command")

        data_length = len(result.data)
        if data_length < 29:
            raise CommandError(
                f"Incomplete drive configuration data: received {data_length} bytes, expected 33"
            )

        # Parse the 33-byte response data structure
        count = result.code
        data = result.data

        try:
            # Convert motor type code to string
            motor_type = (
                "1.8°"
                if data[1] == 0x19
                else "0.9°"
                if data[1] == 0x32
                else f"Unknown (0x{data[2]:02X})"
            )

            # Convert direction to string
            direction = "CW" if data[5] == 0 else "CCW"

            # Calculate subdivision (0 means 256)
            subdivision = 256 if data[6] == 0 else data[6]

            # Convert baud rate code to actual rate
            baud_rates = {1: 9600, 2: 19200, 3: 38400, 4: 57600, 5: 115200}
            baud_rate = baud_rates.get(data[15], f"Unknown (0x{data[15]:02X})")

            # Convert CAN rate code to actual rate
            can_rates = {
                1: 10000,
                2: 20000,
                3: 50000,
                4: 100000,
                5: 125000,
                6: 250000,
                7: 500000,
                8: 1000000,
            }
            can_rate = can_rates.get(data[16], f"Unknown (0x{data[14]:02X})")

            # Parse stall parameters (6 bytes starting at index 20)
            stall_speed = int.from_bytes(data[21:23], "big")
            stall_current = int.from_bytes(data[23:25], "big")
            stall_time = int.from_bytes(data[25:27], "big")
            position_window = (
                int.from_bytes(data[27:29], "big") / 10.0
            )  # Convert to degrees

            config = {
                "byte_count": count,
                "param_count": data[0],
                "motor_type": motor_type,
                "pulse_control_mode": data[2],
                "communication_mode": data[3],
                "en_pin_level": data[4],
                "dir_pin_direction": direction,
                "subdivision": subdivision,
                "subdivision_interpolation": bool(data[7]),
                "auto_screen_off": bool(data[8]),
                "open_loop_current": int.from_bytes(data[9:11], "big"),  # mA
                "max_closed_loop_current": int.from_bytes(data[11:13], "big"),  # mA
                "max_output_voltage": int.from_bytes(data[13:15], "big"),  # mV
                "uart_baud_rate": baud_rate,
                "can_rate": can_rate,
                "device_id": data[17],
                "checksum": data[18],
                "command_response_mode": "Acknowledge" if data[19] else "Full",
                "stall_protection": bool(data[20]),
                "stall_speed_threshold": stall_speed,  # RPM
                "stall_current_threshold": stall_current,  # mA
                "stall_detection_time": stall_time,  # ms
                "position_arrival_window": position_window,  # degrees
            }

            # Log successful parsing
            self._logger.debug(f"Successfully parsed drive configuration: {config}")

            return config

        except Exception as e:
            self._logger.error(f"Error parsing drive configuration: {e}")
            self._logger.debug(f"Raw data: {data.hex(' ').upper()}")
            raise CommandError(f"Failed to parse drive configuration: {e}")

    def get_system_status(self) -> dict:
        """Get system status parameters"""
        cmd = Command(
            CMD.READ_SYSTEM_STATUS, data=bytes([0x7A]), description="Read system status"
        )
        result = self._execute(cmd)
        if result.success and result.data:
            # Parse the complex status data structure
            # Example implementation - adjust based on actual data format
            return {
                "bus_voltage": int.from_bytes(result.data[0:2], "big") / 1000.0,
                "phase_current": int.from_bytes(result.data[2:4], "big") / 1000.0,
                "encoder_value": int.from_bytes(result.data[4:6], "big"),
                "position": int.from_bytes(result.data[6:10], "big", signed=True),
                "speed": int.from_bytes(result.data[10:12], "big", signed=True),
                "status_flags": result.data[12],
                # Add other status parameters as needed
            }
        raise CommandError("Failed to read system status")

    def get_version(self) -> str:
        """Get firmware version string"""
        cmd = Command(CMD.READ_VERSION, description="Read firmware version")
        result = self._execute(cmd)
        if result.success and result.data:
            # Convert bytes to string, assuming ASCII encoding
            return result.data.decode("ascii").strip()
        raise CommandError("Failed to read firmware version")

    def get_encoder_calibration_status(self) -> EncoderCalibrationStatus:
        """Get detailed encoder calibration status

        Returns:
            EncoderCalibrationStatus object with calibration flags
        """
        # Initialize default values
        is_calibrated = False
        calibration_ready = False
        zero_set = False

        # Try to get homing status
        try:
            homing_status = self.get_homing_status()
            is_calibrated = homing_status["encoder_ready"]
            calibration_ready = homing_status["calibration_ready"]
        except CommandError:
            self._logger.warning("Failed to read homing status")

        # Try to verify zero position
        try:
            _ = self.get_calibrated_encoder()
            zero_set = True
        except CommandError:
            self._logger.warning("Failed to read encoder value")

        return EncoderCalibrationStatus(
            is_calibrated=is_calibrated,
            calibration_table_ready=calibration_ready,
            zero_position_set=zero_set,
        )

    def steps_to_angle(self, steps: int) -> float:
        """Convert steps to angle in degrees

        Args:
            steps: Number of steps

        Returns:
            Angle in degrees
        """
        return (abs(steps) % 65536) * 360 / 65536

    def angle_to_steps(self, angle: float) -> int:
        """Convert angle to nearest number of steps

        Args:
            angle: Angle in degrees

        Returns:
            Number of steps
        """
        return int((angle * 65536) / 360) % 65536

    def move_to_angle(
        self,
        angle: float,
        speed: int,
        direction: Direction = 1,
        acceleration: int = 0,
        relative: bool = True,
        sync: SyncFlag = 0,
    ) -> None:
        """Move motor to specified angle

        Args:
            angle: Target angle in degrees
            speed: Movement speed in RPM
            direction: Rotation direction
            acceleration: Acceleration (0-255)
            relative: If True, move relative to current position
            sync: Sync flag
        """
        steps = self.angle_to_steps(angle)
        self.set_position(
            steps=steps,
            speed=speed,
            direction=direction,
            acceleration=acceleration,
            relative=relative,
            sync=sync,
        )
