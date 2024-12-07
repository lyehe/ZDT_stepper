"""Test the stepper motor controller."""

from time import sleep

import pytest
from serial import Serial

from src.stepper.commands.get import (
    GetConfig,
    GetError,
    GetMotorRH,
    GetVersion,
)
from src.stepper.commands.home import (
    GetHomeStatus,
    Home,
    SetHome,
)
from src.stepper.commands.move import (
    Disable,
    Enable,
    EStop,
    Move,
    SyncMove,
)
from src.stepper.commands.system import (
    ZeroAllPositions,
)
from src.stepper.serial_utilities.serial_utilities import (
    TestCase,
    scan_ports,
)
from src.stepper.stepper_core.configs import (
    AbsoluteFlag,
    Acceleration,
    Address,
    Direction,
    HomingMode,
    PulseCount,
    Speed,
    SyncFlag,
)
from src.stepper.stepper_core.parameters import (
    DeviceParams,
    JogParams,
    PositionParams,
)


@pytest.fixture
def serial_connection():
    """Create a serial connection to the stepper motor."""
    # Scan for available ports first
    ports = scan_ports(test_case=TestCase.default())
    if not ports:
        pytest.skip("No serial ports available")

    port = list(ports.keys())[0]  # Get first available port
    baud_rate = ports[port]

    connection = Serial(port, baud_rate, timeout=0.1)
    yield connection
    connection.close()


@pytest.fixture
def device_params(serial_connection):
    """Create device parameters for the stepper motor."""
    return DeviceParams(
        serial_connection=serial_connection,
        address=Address(0x01),
    )


@pytest.fixture
def position_params():
    """Create position parameters for movement commands."""
    return PositionParams(
        direction=Direction.CW,
        speed=Speed(500),
        acceleration=Acceleration(127),
        pulse_count=PulseCount(160),
        absolute=AbsoluteFlag.RELATIVE,
    )


@pytest.fixture
def jog_params():
    """Create jog parameters."""
    return JogParams(
        direction=Direction.CW,
        speed=Speed(1),
        acceleration=Acceleration(255),
    )


@pytest.fixture(autouse=True)
def motor_state_management(device_params):
    """Fixture to manage motor state before and after each test."""
    # Setup: Ensure motor is in known state
    Disable(device=device_params)
    sleep(0.1)
    Enable(device=device_params)
    sleep(0.1)
    
    yield  # Run the test
    
    # Cleanup: Ensure motor is enabled after test
    try:
        Enable(device=device_params)
        sleep(0.1)
    except Exception as e:
        pytest.fail(f"Failed to enable motor during cleanup: {e}")


def test_connection_setup(serial_connection):
    """Test initial connection setup."""
    assert serial_connection.is_open


def test_basic_movement(device_params, position_params):
    """Test basic movement commands."""
    # Move the motor
    move_status = Move(device=device_params, params=position_params).status
    assert move_status in ["SUCCESS", "CONDITIONAL_ERROR"]
    
    if move_status == "SUCCESS":
        sleep(0.1)  # Allow time for movement
        # Check position and errors
        error = GetError(device=device_params).raw_data.data_dict
        assert abs(error.get('position_error (deg)', float('inf'))) < 1.0


def test_homing_sequence(device_params):
    """Test homing functionality."""
    # Set home position
    home_status = SetHome(device=device_params).status
    assert home_status in ["SUCCESS", "CONDITIONAL_ERROR"]
    
    # Zero all positions
    zero_all_positions = ZeroAllPositions
    zero_all_positions.unlock()
    zero_status = zero_all_positions(device=device_params).status
    assert zero_status in ["SUCCESS", "CONDITIONAL_ERROR"]
    
    # Perform homing
    homing_status = Home(device=device_params, params=HomingMode.MULTI_TURN_UNLIMITED).status
    assert homing_status in ["SUCCESS", "CONDITIONAL_ERROR"]
    
    if homing_status == "SUCCESS":
        sleep(0.1)
        # Check home status
        home_status = GetHomeStatus(device=device_params).raw_data.data_dict
        assert home_status['encoder_ready'] is True
        assert home_status['encoder_calibrated'] is True


def test_system_status(device_params):
    """Test system status and configuration retrieval."""
    # Get various status information
    version = GetVersion(device=device_params).data
    assert isinstance(version["firmware_version"], int)
    assert isinstance(version["hardware_version"], int)

    motor_params = GetMotorRH(device=device_params).data
    assert isinstance(motor_params["phase_resistance (mOhm)"], float)
    assert isinstance(motor_params["phase_inductance (uH)"], float)

    config = GetConfig(device=device_params).data
    assert isinstance(config["stepper_type"], str)
    assert isinstance(config["control_mode"], str)


def test_error_conditions(device_params, position_params):
    """Test error conditions and motor response."""
    # Disable motor and try to move (should fail)
    status = Disable(device=device_params).status
    assert status in ["SUCCESS", "CONDITIONAL_ERROR"]

    status = Move(device=device_params, params=position_params).status
    assert status == "CONDITIONAL_ERROR"

    # Try emergency stop
    assert EStop(device=device_params).status in ["SUCCESS", "CONDITIONAL_ERROR"]


def test_sync_movement(device_params, position_params):
    """Test synchronized movement functionality."""
    # Enable motor first
    status = Enable(device=device_params).status
    assert status in ["SUCCESS", "CONDITIONAL_ERROR"]

    # Test sync move
    status = Move(device=device_params, params=position_params, setting=SyncFlag.SYNC).status
    assert status in ["SUCCESS", "CONDITIONAL_ERROR", "ERROR"]

    # Test sync move
    result = SyncMove(device=device_params).status
    assert result in ["SUCCESS", "CONDITIONAL_ERROR", "ERROR"]
