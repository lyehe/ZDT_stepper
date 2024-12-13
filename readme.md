## Quick Start
This is a Python library for controlling the closed-loop stepper motor controller from ZDT stepper（张大头步进电机） via UART serial connection. It includes all the documented commands from the spec sheet, and handles all parameters in a type-safe manner. It can also be extended with additional functionality, CAN communication, and can be modified for other serial controller devices. Fully tested and compatible with Python 3.10 and above.


### Installation
```bash
pip install zdt_stepper
```


### Basic Example
```python
from serial import Serial
from stepper.device import Device
from stepper.stepper_core.parameters import DeviceParams
from stepper.stepper_core.configs import Address

# Connect to motor
serial = Serial("COM4", 115200, timeout=0.1)
device = Device(
    device_params=DeviceParams(
        serial_connection=serial,
        address=Address(0x01)
    )
)

# Basic movement controls
device.enable()
device.move_cw(1000)  # Move 1000 pulses clockwise
device.move_to(0)     # Move to absolute position 0
device.jog(500)       # Continuous movement at 500 pulses/sec
device.stop()         # Stop movement
```

### Features
- Comprehensive motor control (enable/disable, absolute/relative moves, jogging)
- Advanced movement profiles with acceleration control
- Real-time position and status monitoring
- PID tuning and closed-loop control
- Configuration management (microsteps, currents, protection settings)
- Homing and calibration routines
- Command string interface for simple control
- Extensive error handling and debugging

### Status Monitoring
```python
# Get various status information
print(f"Position: {device.real_time_position.position}")
print(f"Speed: {device.real_time_speed.speed}")
print(f"Status: Enabled={device.is_enabled}, In Position={device.is_in_position}")
print(f"Voltage: {device.bus_voltage.voltage}")
print(f"Current: {device.phase_current.current}")
```

### Configuration
```python
# Configure motor parameters
device.set_microstep(32)
device.set_open_loop_current(1000)
device.set_pid(p=100, i=50, d=20)
device.set_speed(1000)
device.set_acceleration(500)

# Save settings permanently
device.enable_store()
device.set_config()
```

### Command Interface (under development)
```python
# Use simple command strings for control
device.parse_cmd("HOM")          # Home the motor
device.parse_cmd("MOV 1000")     # Move to position 1000
device.parse_cmd("JOG 500")      # Jog at 500 pulses/sec
device.parse_cmd("STP")          # Stop movement
device.parse_cmd("PID 100 50 20") # Set PID parameters
```

### The Command Object Interface (for debugging)
```python
from serial import Serial
from stepper.commands.move import Enable, Move
from stepper.stepper_core.parameters import DeviceParams, PositionParams
from stepper.stepper_core.configs import (
    Address, Direction, Speed,
    Acceleration, PulseCount, AbsoluteFlag
)

# Connect to motor
serial = Serial("COM4", 115200, timeout=0.1)
device = DeviceParams(
    serial_connection=serial,
    address=Address(0x01)
)

# Enable motor
Enable(device=device).status
# Configure movement
params = PositionParams(
    direction=Direction.CW,
    speed=Speed(500),
    acceleration=Acceleration(127),
    pulse_count=PulseCount(160),
    absolute=AbsoluteFlag.RELATIVE
)

# Move motor
Move(device=device, params=params).status

# Move to absolute position
params.absolute = AbsoluteFlag.ABSOLUTE
Move(device=device, params=params).status
```
### Status Monitoring
```python
from stepper.commands.get import GetSysStatus

status = GetSysStatus(device=device).raw_data.data_dict
print(status)
```

### Configuration
```python
from src.stepper.commands.set import SetConfig
from src.stepper.stepper_constants import Microstep, StallTime

config = GetConfig(device=device).raw_data
config.microstep = Microstep(value=32)
config.stall_time = StallTime(value=100)
SetConfig(device=device, params=config).status
```

### Development
```bash
# Install dev dependencies
pip install zdt_stepper[dev]
```

### List of all functions under the device class

System Information
- init_time
- version
- motor_rh
- bus_voltage
- phase_current
- encoder_value
- pulse_count
- target_position
- open_loop_setpoint
- real_time_speed
- real_time_position
- position_error
- sys_status
- status

Status Flags
- is_enabled
- is_in_position
- is_stalled
- is_stall_protection_active
- encoder_ready
- encoder_calibrated
- is_homing
- is_homing_failed
- is_sync
- is_store

Movement Parameters
- jog_direction
- jog_speed
- jog_acceleration
- move_speed
- move_acceleration
- move_direction
- move_pulse_count
- move_mode

Configuration
- pid
- config
- homing_params
- homing_status
- state_dict
- params_dict

PID Control
- set_pid()
- set_p()
- set_i()
- set_d()

Device Configuration
- set_config()
- set_stepper_type()
- set_control_mode()
- set_communication_mode()
- set_enable_level()
- set_default_direction()
- set_microstep()
- set_microstep_interp()
- set_screen_off()
- set_open_loop_current()
- set_max_closed_loop_current()
- set_max_voltage()
- set_baud_rate()
- set_canrate()
- set_id()
- set_checksum_mode()
- set_response_mode()
- set_stall_protect()
- set_stall_speed()
- set_stall_current()
- set_stall_time()
- set_on_target_window()

Start Speed Configuration
- set_start_speed_params()
- set_start_direction()
- set_start_speed()
- set_start_acceleration()
- set_start_en_control()

System Configuration
- set_loop_mode()
- set_speed_reduction()

System Commands
- sys_calibrate_encoder()
- sys_factory_reset()
- sys_clear_stall()
- sys_zero_all_positions()

Homing Methods
- set_homing_params()
- set_homing_mode()
- set_homing_direction()
- set_homing_speed()
- set_homing_timeout()
- set_collision_detection_speed()
- set_collision_detection_current()
- set_collision_detection_time()
- set_auto_home()
- home()
- set_home()
- stop_home()

Movement Control
- enable()
- disable()
- estop()
- jog()
- jog_cw()
- jog_ccw()
- jog_at_speed()
- set_jog_speed()
- set_jog_direction()
- set_jog_acceleration()
- stop()
- move()
- move_to()
- move_cw()
- move_ccw()
- set_move_speed()
- set_move_direction()
- set_move_acceleration()
- set_speed()
- set_direction()
- set_acceleration()
- sync_move()

Sync and Store Control
- enable_sync()
- disable_sync()
- enable_store()
- disable_store()

Utility Methods
- wait()
- debug()
- resolve_bug()
- tic()
- toc()
- jog_time()
- parse_cmd()

## License
MIT License
