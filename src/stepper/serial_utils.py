"""Serial port utilities for testing and detecting connections.

Example usage:
    # List available ports
    python serial_utils.py -l

    # Show detailed port info
    python serial_utils.py -i

    # Test connection on COM3 at 115200 baud
    python serial_utils.py -p COM3 -b 115200 -t

    # Detect baudrate for COM3
    python serial_utils.py -p COM3 -d

    # Scan all ports using custom test case
    python serial_utils.py -s -tc my_test_case.yaml

    # Detect port at 115200 baud
    python serial_utils.py -b 115200 -dp

:raises FileNotFoundError: If test case YAML file is not found
:return: Various functions for testing serial connections
:rtype: None
"""

from dataclasses import dataclass
from pathlib import Path
import serial
import time
from serial.tools import list_ports
from logging import getLogger
from tqdm import tqdm
import yaml
import argparse

logger = getLogger(__name__)

PathVar = Path | str
BAUDRATES = (9600, 115200, 19200, 38400, 57600)


@dataclass
class TestCase:
    input: str
    expected: str
    check_digit: int | tuple[int, ...] | None = None
    timeout: float = 0.1

    @property
    def input_bytes(self) -> bytes:
        """Convert hex string to bytes"""
        return bytes.fromhex(self.input)

    @property
    def expected_bytes(self) -> bytes:
        """Convert hex string to bytes"""
        return bytes.fromhex(self.expected)

    @classmethod
    def from_yaml(cls, path: PathVar) -> "TestCase":
        path = Path(path)
        if not path.exists():
            logger.error(f"File not found: {path}")
            raise FileNotFoundError(str(path))
        with open(path, "r") as f:
            data = yaml.safe_load(f)
        return cls(**data)


@dataclass
class PortInfo:
    device: str
    description: str
    vid: int | None
    pid: int | None
    manufacturer: str | None
    serial_number: str | None
    location: str | None


def list_ports_info() -> dict[str, PortInfo]:
    """Get available serial ports with detailed metadata

    :return: Dictionary mapping port device names to PortInfo objects
    """
    return {
        port.device: PortInfo(
            device=port.device,
            description=port.description,
            vid=port.vid,
            pid=port.pid,
            manufacturer=port.manufacturer,
            serial_number=port.serial_number,
            location=port.location,
        )
        for port in list_ports.comports()
    }


@staticmethod
def test_connection(
    port: str,
    baudrate: int = 115200,
    timeout: float = 1,
    test_case: TestCase | None = None,
) -> bool:
    """Test connection to port

    :param port: Serial port device name
    :param baudrate: Communication speed in baud
    :param timeout: Read timeout in seconds
    :param test_case: Optional test case to verify communication
    :param check_digit: Optional index of digit to check in output
    :return: True if connection successful and test passes
    """
    try:
        with serial.Serial(port, baudrate, timeout=timeout) as ser:
            logger.debug(f"Connected to {port} at {baudrate} baud")
            if test_case:
                ser.write(test_case.input_bytes)
                output = ser.readline()  # Read raw bytes
                if test_case.check_digit is not None:
                    logger.debug(f"Checking digit(s) {test_case.check_digit}")
                    if isinstance(test_case.check_digit, tuple):
                        return all(
                            len(output) > idx
                            and output[idx] == test_case.expected_bytes[idx]
                            for idx in test_case.check_digit
                        )
                    elif isinstance(test_case.check_digit, int):
                        return (
                            len(output) > test_case.check_digit
                            and output[test_case.check_digit]
                            == test_case.expected_bytes[test_case.check_digit]
                        )
                return output == test_case.expected_bytes
            return True
    except serial.SerialException as e:
        logger.error(f"Serial connection error: {e}")
        return False
    except Exception as e:
        logger.error(f"Unexpected error: {e}")
        return False


def detect_baudrate(port: str, test_case: TestCase) -> int | None:
    """Find working baudrate for port

    :param port: Serial port device name
    :param test_case: Test case to verify communication
    :return: Working baudrate if found, None otherwise
    """
    for baud in tqdm(BAUDRATES, desc=f"Testing baudrates on {port}"):
        if test_connection(port, baud, test_case.timeout, test_case):
            logger.debug(f"Found working baudrate: {baud}")
            return baud
        logger.debug(f"Failed to connect to {port} at {baud} baud")
        time.sleep(0.05)
    return None


def detect_port(baudrate: int, test_case: TestCase) -> str | None:
    """Detect port with working baudrate

    :param baudrate: Communication speed in baud
    :param test_case: Test case to verify communication
    :return: Port device name if found, None otherwise
    """
    for port in list_ports.comports():
        if test_connection(port.device, baudrate, test_case.timeout, test_case):
            return port.device
    return None


def scan_ports(test_case: TestCase) -> dict[str, int]:
    """Scan ports for working baudrate and return dictionary of port device names and their corresponding baudrates

    :param test_case: Test case to verify communication
    :return: Dictionary mapping port device names to baudrates
    """
    success = {}
    ports = list(list_ports.comports())
    for port in ports:
        for baudrate in tqdm(BAUDRATES, desc=f"Testing {port.device}", leave=False):
            if test_connection(port.device, baudrate, test_case.timeout, test_case):
                success[port.device] = baudrate
                break
    return success


def print_ports() -> None:
    """List port names"""
    print(f"Available ports: {[port.device for port in list_ports.comports()]}")


def print_ports_info(ports_info: dict[str, PortInfo]) -> None:
    """Print port information"""
    print("-" * 125)
    print(
        f"{'Port':<10} {'Description':<30} {'VID':<5} {'PID':<5} {'Manufacturer':<25} {'Serial Number':<25} {'Location':<25}"
    )
    print("-" * 125)

    for port, info in ports_info.items():
        description = info.description if info.description else "N/A"
        manufacturer = info.manufacturer if info.manufacturer else "N/A"
        vid = info.vid if info.vid else "N/A"
        pid = info.pid if info.pid else "N/A"
        serial_number = info.serial_number if info.serial_number else "N/A"
        location = info.location if info.location else "N/A"
        print(
            f"{port:<10} {description:<30} {vid:<5} {pid:<5} {manufacturer:<25} {serial_number:<25} {location:<25}"
        )
    print("-" * 125)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Serial port utilities")
    parser.add_argument(
        "--port", "-p", type=str, default="", help="Serial port to use"
    )
    parser.add_argument(
        "--baudrate", "-b", type=int, default=115200, help="Baudrate to use"
    )
    parser.add_argument(
        "--list", "-l", action="store_true", help="List available ports"
    )
    parser.add_argument(
        "--info", "-i", action="store_true", help="Show detailed port info"
    )
    parser.add_argument("--test", "-t", action="store_true", help="Test connection")
    parser.add_argument("--scan", "-s", action="store_true", help="Scan all ports")
    parser.add_argument(
        "--detect-baudrate", "-d", action="store_true", help="Detect baudrate"
    )
    parser.add_argument("--detect-port", "-dp", action="store_true", help="Detect port")
    parser.add_argument(
        "--test-case",
        "-tc",
        type=str,
        required=False,
        default=str(Path(__file__).parent / "default_serial_test_case.yaml"),
        help="Path to YAML file containing test case parameters",
    )
    args = parser.parse_args()

    test_case = TestCase.from_yaml(args.test_case)

    if args.list:
        print_ports()

    if args.info:
        print_ports_info(list_ports_info())

    if args.test:
        result = test_connection(
            args.port,
            baudrate=args.baudrate,
            test_case=test_case,
        )
        print(f"Connection test {'successful' if result else 'failed'}")

    if args.detect_baudrate:
        baudrate = detect_baudrate(args.port, test_case)
        print(f"Detected baudrate: {baudrate}")

    if args.detect_port:
        port = detect_port(args.baudrate, test_case)
        print(f"Detected port: {port}")

    if args.scan:
        scan_results = scan_ports(test_case)
        print(f"Scan results: {scan_results}")
