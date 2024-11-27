import pytest
from unittest.mock import Mock, patch
from src.stepper import StepperMotor

@pytest.fixture
def motor():
    with patch('serial.Serial') as mock_serial:
        # Create a mock serial instance
        serial_instance = mock_serial.return_value
        
        # Configure mock serial port
        serial_instance.is_open = True
        
        # Create a mock read method
        mock_read = Mock(return_value=b'\x00')  # Default success response
        serial_instance.read = mock_read
        
        motor = StepperMotor("COM5")
        yield motor

def test_enable(motor):
    motor.enable()
    assert motor.is_enabled

def test_set_speed(motor):
    # Enable the motor first
    motor.enable()
    
    # Mock successful response for set_speed command
    motor._serial.read = Mock(return_value=b'\x00')  # Success response
    
    # Test setting speed
    motor.set_speed(1000, direction=1, acceleration=10)
    
    # Verify the motor is still enabled after setting speed
    assert motor.is_enabled