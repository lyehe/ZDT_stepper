"""StepperLib - A robust library for stepper motor control
"""

import logging
import time

from serial import Serial

from .stepper_constants import (
    CMD_CODE,
    PROTOCOL,
    BaudRate,
    COMPort,
    Direction,
    StoreFlag,
    SyncFlag,
)
from .stepper_dataclasses import (
    CommandInput,
    CommandOutput,
    EncoderCalibrationStatus,
    MotorStatus,
    Position,
)
from .stepper_exceptions import (
    CommandError,
    CommunicationError,
    StatusError,
    ValidationError,
)


class SerialController:
    """Main stepper motor control class"""

    def __init__(
        self,
        port: COMPort,
        baudrate: BaudRate = 115200,
        timeout: float = 0.1,
        retry_count: int = 3,
        retry_delay: float = 0.1,
    ):
        self.timeout = timeout
        self._enabled = self.get_status().enabled
        self.retry_count = retry_count
        self.retry_delay = retry_delay
        self._serial = Serial(
            port=str(port), baudrate=baudrate, timeout=timeout, write_timeout=timeout
        )
        self._logger = logging.getLogger("Stepper")

    @property
    def is_enabled(self) -> bool:
        return self._enabled

    def close(self) -> None:
        """Close serial connection"""
        self._serial.close()

    def __enter__(self) -> "SerialController":
        return self

    def __exit__(self) -> None:
        self.close()

    def _execute(self, command: CommandInput) -> CommandOutput:
        """Execute a command and return the result"""
        if command.requires_enabled and not self.is_enabled:
            raise StatusError("Motor must be enabled")

        cmd_bytes = command.build(self.address)
        retry_count = self.retry_count
        self._logger.debug(f"Command: {command.description}")
        self._logger.debug(f"TX: {cmd_bytes.hex(' ').upper()}")

        while retry_count >= 0:
            try:
                self._serial.write(cmd_bytes)
                response = self._serial.read_until(size=command.expected_response_length)
                if not response:
                    raise CommunicationError("No response received")

                self._logger.debug(f"Response to {command.description}:\n")
                self._logger.debug(f"RX: {response.hex(' ').upper()}")

                output = command.parse_response(response)
                self._validate(command, output)
                return output

            except Exception as ex:
                if retry_count == 0:
                    raise
                retry_count -= 1
                self._logger.warning(f"Command {command.description} failed, retrying: {ex}")
                time.sleep(command.retry_delay)

    def _validate(self, command: CommandInput, output: CommandOutput) -> None:
        pass

    def enable(self) -> None:
        """Enable the motor"""
        cmd = CommandInput(
            device_number=self.address,
            code=CMD_CODE.ENABLE,
            data=bytes([CMD_CODE.ENABLE_CODE, PROTOCOL.ENABLE, self._sync]),
            requires_enabled=False,
            description=f"Enable motor (store={self._store}, sync={self._sync})",
            checksum_type=self._checksum_type,
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
        cmd = self._create_command(
            code=CMD_CODE.ENABLE,
            data=bytes([PROTOCOL.SYNC_BYTE, 0, 0]),
            requires_enabled=False,
            description="Disable motor",
        )
        result = self._execute(cmd)
        if result.success:
            self._enabled = False

    def emergency_stop(self) -> None:
        """Emergency stop the motor"""
        cmd = self._create_command(
            CMD_CODE.EMERGENCY_STOP,
            data=bytes([PROTOCOL.EMERGENCY_STOP_CODE, 0]),
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

        cmd = self._create_command(
            CMD_CODE.SPEED,
            data=bytes([direction]) + rpm.to_bytes(2, "big") + bytes([acceleration, sync]),
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

        cmd = self._create_command(
            CMD_CODE.POSITION,
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
        cmd = self._create_command(CMD_CODE.READ_POSITION, description="Read current position")
        result = self._execute(cmd)
        if result.success and result.data:
            return Position.from_bytes(result.data)

        raise CommandError("Failed to read position")

    def get_status(self) -> MotorStatus:
        """Get motor status"""
        cmd = self._create_command(CMD_CODE.READ_STATUS, description="Read motor status")
        result = self._execute(cmd)
        if result.success and result.data:
            return MotorStatus.from_byte(result.data[0])
        raise CommandError("Failed to read status")

    def set_zero(self, store: StoreFlag = 1) -> None:
        """Set current position as zero"""
        cmd = self._create_command(
            CMD_CODE.SET_ZERO,
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

        cmd = self._create_command(
            CMD_CODE.TRIGGER_HOME,
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
        cmd = self._create_command(
            CMD_CODE.ABORT_HOME,
            data=bytes([0x48]),
            description="Abort homing operation",
        )
        result = self._execute(cmd)
        if result.is_condition_error:
            raise StatusError("No homing in progress")

    def clear_stall(self) -> None:
        """Clear stall protection"""
        cmd = self._create_command(
            CMD_CODE.CLEAR_STALL,
            data=bytes([0x52]),
            description="Clear stall protection",
        )
        result = self._execute(cmd)
        if result.is_condition_error:
            raise StatusError("No stall protection triggered")

    def get_realtime_target(self) -> Position:
        """Get real-time target position"""
        cmd = self._create_command(
            CMD_CODE.READ_REALTIME_TARGET, description="Read real-time target position"
        )
        result = self._execute(cmd)
        if result.success and result.data:
            return Position.from_bytes(result.data)
        raise CommandError("Failed to read target position")

    def get_realtime_speed(self) -> int:
        """Get real-time speed in RPM"""
        cmd = self._create_command(CMD_CODE.READ_REALTIME_SPEED, description="Read real-time speed")
        result = self._execute(cmd)
        if result.success and result.data:
            sign = -1 if result.data[0] else 1
            speed = int.from_bytes(result.data[1:3], "big")
            return sign * speed
        raise CommandError("Failed to read speed")

    def get_position_error(self) -> int:
        """Get position error in steps"""
        cmd = self._create_command(CMD_CODE.READ_POSITION_ERROR, description="Read position error")
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
        cmd = self._create_command(
            CMD_CODE.READ_HOMING_STATUS,
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

        cmd = self._create_command(
            CMD_CODE.MODIFY_SUBDIVISION,
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

        cmd = self._create_command(
            CMD_CODE.MODIFY_ID_ADDRESS,
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
        cmd = self._create_command(
            CMD_CODE.SWITCH_LOOP_MODE,
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

        cmd = self._create_command(
            CMD_CODE.MODIFY_OPEN_LOOP_CURRENT,
            data=bytes([0x33, store]) + current_ma.to_bytes(2, "big"),
            description=f"Set open loop current to {current_ma}mA",
        )
        self._execute(cmd)

    def set_pid_parameters(self, kp: int, ki: int, kd: int, store: StoreFlag = 1) -> None:
        """Set position PID parameters

        Args:
            kp: Proportional gain (32-bit)
            ki: Integral gain (32-bit)
            kd: Derivative gain (32-bit)
            store: Store to memory flag
        """
        cmd = self._create_command(
            CMD_CODE.MODIFY_PID_PARAMS,
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

        cmd = self._create_command(
            CMD_CODE.STORE_SPEED_PARAMS,
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
        cmd = self._create_command(
            CMD_CODE.MODIFY_SPEED_SCALE,
            data=bytes([0x71, store, 1 if divide_by_ten else 0]),
            description=f"Set speed scale ({'÷10' if divide_by_ten else 'normal'})",
        )
        self._execute(cmd)

    def get_phase_parameters(self) -> dict[str, float]:
        """Get motor phase parameters

        Returns:
            Dictionary with resistance (mΩ) and inductance (μH)
        """
        cmd = self._create_command(CMD_CODE.READ_PHASE_PARAMS, description="Read phase parameters")
        result = self._execute(cmd)
        if result.success and result.data:
            resistance = int.from_bytes(result.data[0:2], "big")  # mΩ
            inductance = int.from_bytes(result.data[2:4], "big")  # μH
            return {"resistance": resistance, "inductance": inductance}
        raise CommandError("Failed to read phase parameters")

    def get_bus_voltage(self) -> float:
        """Get bus voltage in volts"""
        cmd = self._create_command(CMD_CODE.READ_BUS_VOLTAGE, description="Read bus voltage")
        result = self._execute(cmd)
        if result.success and result.data:
            voltage_mv = int.from_bytes(result.data, "big")
            print(f"Voltage: {result.data}")
            return voltage_mv / 1000.0  # Convert mV to V
        raise CommandError("Failed to read bus voltage")

    def get_phase_current(self) -> float:
        """Get phase current in amps"""
        cmd = self._create_command(CMD_CODE.READ_PHASE_CURRENT, description="Read phase current")
        result = self._execute(cmd)
        if result.success and result.data:
            current_ma = int.from_bytes(result.data, "big")
            return current_ma / 1000.0  # Convert mA to A
        raise CommandError("Failed to read phase current")

    def get_calibrated_encoder(self) -> int:
        """Get calibrated encoder value (0-65535 per revolution)"""
        cmd = self._create_command(
            CMD_CODE.READ_CALIBRATED_ENCODER,
            description="Read calibrated encoder value",
        )
        result = self._execute(cmd)
        if result.success and result.data:
            return int.from_bytes(result.data, "big")
        raise CommandError("Failed to read encoder value")

    def get_command_body_pulse_count(self) -> int:
        """Get input pulse count with direction"""
        cmd = self._create_command(
            CMD_CODE.READ_command_body_PULSE_COUNT, description="Read input pulse count"
        )
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
            - stall_protection_active: True/False
            - stall_speed_threshold: Speed in RPM
            - stall_current_threshold: Current in mA
            - stall_detection_time: Time in ms
            - position_arrival_window: Window in degrees
        """
        cmd = self._create_command(
            CMD_CODE.READ_DRIVE_CONFIG,
            data=bytes([0x6C]),
            description="Read drive configuration",
            requires_enabled=False,
        )
        result = self._execute(cmd)

        if not result.success or not result.data:
            raise CommandError("Failed to read drive configuration")

        try:
            data = result.data

            # Debug log the raw data
            self._logger.debug(f"Drive config raw data: {data.hex(' ').upper()}")

            # Basic validation
            if len(data) < 29:  # Minimum expected length
                raise CommandError(f"Incomplete configuration data: {len(data)} bytes")

            # Parse fixed fields
            config = {
                "motor_type": "1.8°"
                if data[0] == 0x19
                else "0.9°"
                if data[0] == 0x32
                else f"Unknown (0x{data[0]:02X})",
                "pulse_control_mode": data[1],
                "communication_mode": data[2],
                "en_pin_level": data[3],
                "dir_pin_direction": "CW" if data[4] == 0 else "CCW",
                "subdivision": 256 if data[5] == 0 else data[5],
                "subdivision_interpolation": bool(data[6]),
                "auto_screen_off": bool(data[7]),
                "open_loop_current": int.from_bytes(data[8:10], "big"),
                "max_closed_loop_current": int.from_bytes(data[10:12], "big"),
                "max_output_voltage": int.from_bytes(data[12:14], "big"),
                "uart_baud_rate": {
                    1: 9600,
                    2: 19200,
                    3: 38400,
                    4: 57600,
                    5: 115200,
                }.get(data[14], f"Unknown (0x{data[14]:02X})"),
                "can_rate": {
                    1: 10000,
                    2: 20000,
                    3: 50000,
                    4: 100000,
                    5: 125000,
                    6: 250000,
                    7: 500000,
                    8: 1000000,
                }.get(data[15], f"Unknown (0x{data[15]:02X})"),
                "device_id": data[16],
                "checksum": data[17],
                "command_response_mode": "Acknowledge" if data[18] else "Full",
                "stall_protection_active": bool(data[19]),
                "stall_speed_threshold": int.from_bytes(data[20:22], "big"),
                "stall_current_threshold": int.from_bytes(data[22:24], "big"),
                "stall_detection_time": int.from_bytes(data[24:26], "big"),
                "position_arrival_window": int.from_bytes(data[26:28], "big") / 10.0,
            }

            return config

        except Exception as e:
            self._logger.error(f"Error parsing drive configuration: {e}")
            self._logger.debug(f"Raw data: {data.hex(' ').upper()}")
            raise CommandError(f"Failed to parse drive configuration: {e}")

    def get_system_status(self) -> dict:
        """Get comprehensive system status parameters

        Returns:
            Dictionary containing:
            - bus_voltage: Bus voltage in volts
            - phase_current: Phase current in amps
            - encoder_value: Raw encoder value (0-65535)
            - target_position: Target position object (steps, revolutions, degrees)
            - realtime_speed: Current speed in RPM
            - realtime_position: Current position object (steps, revolutions, degrees)
            - position_error: Position error object (steps, degrees)
            - ready_status: Dictionary of ready status flags
            - motor_status: Dictionary of motor status flags
        """
        cmd = self._create_command(
            CMD_CODE.READ_SYSTEM_STATUS,
            data=bytes([0x7A]),
            description="Read system status",
        )
        result = self._execute(cmd)

        if not result.success or not result.data:
            raise CommandError("Failed to read system status")

        try:
            data = result.data

            # Debug log the raw data
            self._logger.debug(f"System status raw data: {data.hex(' ').upper()}")

            # Parse each field
            bus_voltage = int.from_bytes(data[0:2], "big") / 1000.0
            phase_current = int.from_bytes(data[2:4], "big") / 1000.0
            encoder_value = int.from_bytes(data[4:6], "big")

            # Target position (5 bytes: sign + value)
            target_sign = -1 if data[6] else 1
            target_steps = target_sign * int.from_bytes(data[7:11], "big")
            target_position = Position(
                steps=target_steps,
                revolutions=target_steps // 65536,
                degrees=(abs(target_steps) % 65536) * 360 / 65536,
            )

            # Real-time speed (3 bytes: sign + value)
            speed_sign = -1 if data[11] else 1
            realtime_speed = speed_sign * int.from_bytes(data[12:14], "big")

            # Real-time position (5 bytes: sign + value)
            position_sign = -1 if data[14] else 1
            position_steps = position_sign * int.from_bytes(data[15:19], "big")
            realtime_position = Position(
                steps=position_steps,
                revolutions=position_steps // 65536,
                degrees=(abs(position_steps) % 65536) * 360 / 65536,
            )

            # Position error (5 bytes: sign + value)
            error_sign = -1 if data[19] else 1
            error_steps = error_sign * int.from_bytes(data[20:24], "big")

            # Status flags
            ready_status = data[24]
            motor_status = data[25]

            return {
                "bus_voltage": bus_voltage,
                "phase_current": phase_current,
                "encoder_value": encoder_value,
                "target_position": target_position,
                "realtime_speed": realtime_speed,
                "realtime_position": realtime_position,
                "position_error": {
                    "steps": error_steps,
                    "degrees": (abs(error_steps) % 65536) * 360 / 65536,
                },
                "ready_status": {
                    "encoder_ready": bool(ready_status & 0x01),
                    "calibration_ready": bool(ready_status & 0x02),
                    "homing_in_progress": bool(ready_status & 0x04),
                    "homing_failed": bool(ready_status & 0x08),
                },
                "motor_status": {
                    "enabled": bool(motor_status & 0x01),
                    "position_reached": bool(motor_status & 0x02),
                    "stalled": bool(motor_status & 0x04),
                    "protection_triggered": bool(motor_status & 0x08),
                },
            }

        except Exception as e:
            self._logger.error(f"Error parsing system status: {e}")
            self._logger.debug(f"Raw data: {data.hex(' ').upper()}")
            raise CommandError(f"Failed to parse system status: {e}")

    def get_version(self) -> str:
        """Get firmware version string"""
        cmd = self._create_command(CMD_CODE.READ_VERSION, description="Read firmware version")
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

    def factory_reset(self) -> None:
        """Reset to factory settings"""
        cmd = self._create_command(
            CMD_CODE.FACTORY_RESET,
            data=bytes([0x5F]),
            requires_enabled=False,
            description="Factory reset",
        )
        self._execute(cmd)
        self._enabled = False
