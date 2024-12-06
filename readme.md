## Quick Start

### Installation
```bash
pip install zdt_stepper
```

### Basic Example
```python
from serial import Serial
from src.stepper.commands.move import Enable, Move
from src.stepper.stepper_core.parameters import DeviceParams, PositionParams
from src.stepper.stepper_core.configs import (
    Address, Direction, Speed, 
    Acceleration, PulseCount, AbsoluteFlag
)

# Connect to motor
serial = Serial("COM4", 115200, timeout=0.1)
device = DeviceParams(
    serial_connection=serial,
    address=Address(0x01)
)

# Configure movement
params = PositionParams(
    direction=Direction.CW,
    speed=Speed(500),
    acceleration=Acceleration(127),
    pulse_count=PulseCount(160),
    absolute=AbsoluteFlag.RELATIVE
)

# Move motor
Enable(device=device).status
Move(device=device, params=params).status
```

### Features
- Motor control (enable/disable, move, jog, e-stop)
- Homing and calibration
- Real-time status monitoring
- Configuration management
- PID tuning
- Error handling

### Status Monitoring
```python
from src.stepper.commands.get import GetSysStatus

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

## License
MIT License
