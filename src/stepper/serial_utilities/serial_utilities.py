"""Serial port utilities for testing and detecting connections.

Example usage:
    # List available ports
    python serial_utils.py --list

    # Show detailed port info
    python serial_utils.py --info

    # Test connection on COM3 at 115200 baud
    python serial_utils.py --port COM3 --baudrate 115200 --test

    # Detect baudrate for COM3
    python serial_utils.py --port COM3 --detect-baudrate

    # Scan all ports using custom test case
    python serial_utils.py --scan --test-case my_test_case.yaml

    # Detect port at 115200 baud
    python serial_utils.py --baudrate 115200 --detect-port

:raises FileNotFoundError: If test case YAML file is not found
:return: Various functions for testing serial connections
:rtype: None
"""

import argparse
import logging
import re
import time
from dataclasses import dataclass, field
from pathlib import Path

import serial
import yaml
from serial.tools import list_ports
from tqdm import tqdm

logger = logging.getLogger(__name__)

PathVar = Path | str
BAUDRATES = (9600, 115200, 19200, 38400, 57600)

__all__ = [
    "BAUDRATES",
    "PathVar",
    "PortInfo",
    "TestCase",
    "detect_baudrate",
    "detect_port",
    "list_ports_info",
    "print_ports",
    "print_ports_info",
    "scan_ports",
    "test_connection",
]


@dataclass
class TestCase:
    """Test case for serial communication."""

    input: str
    expected: str
    check_digit: int | list[int] | None = None
    timeout: float = 0.1

    _command_body_bytes: bytes | None = field(init=False)
    _expected_bytes: bytes | None = field(init=False)

    def __post_init__(self):
        """Post-initialization checks."""
        # Validate input and expected can be converted to bytes
        try:
            self._command_body_bytes = bytes.fromhex(self.input)
        except ValueError as err:
            raise ValueError(f"Invalid input hex string: {err}") from err

        try:
            self._expected_bytes = bytes.fromhex(self.expected)
        except ValueError as err:
            raise ValueError(f"Invalid expected hex string: {err}") from err

        if self.check_digit is not None:
            if isinstance(self.check_digit, int):
                self.check_digit = [self.check_digit]
            if len(self.expected) < max(self.check_digit):
                logger.warning(
                    f"Expected length {len(self.expected)} < "
                    f"max check digit {max(self.check_digit)}"
                )
                self.check_digit = None

    @classmethod
    def default(cls) -> "TestCase":
        """Get default test case.

        :return: Default TestCase instance
        """
        return cls(input="011F6B", expected="011FFF786B", check_digit=[1], timeout=0.1)

    @property
    def input_bytes(self) -> bytes:
        """Convert hex string to bytes."""
        return self._command_body_bytes

    @property
    def expected_bytes(self) -> bytes:
        """Convert hex string to bytes."""
        return self._expected_bytes

    @classmethod
    def from_yaml(cls, path: PathVar) -> "TestCase":
        """Load test case from YAML file."""
        path = Path(path)
        if not path.exists():
            logger.error(f"File not found: {path}")
            raise FileNotFoundError(str(path))
        with open(path) as f:
            data = yaml.safe_load(f)
        return cls(**data)

    @classmethod
    def from_str(cls, data: str) -> "TestCase":
        """Load test case from string."""
        # Split string by delimiters and parse as YAML
        if Path(data).exists():
            return cls.from_yaml(data)
        try:
            parts = [part for part in re.split(r"[/\\|&+,\s]", data) if part]
            if len(parts) != 2:
                raise ValueError("Input string must contain exactly one delimiter")
            return cls(input=parts[0], expected=parts[1])
        except ValueError as err:
            raise ValueError(f"Invalid input string: {err}") from err


@dataclass
class PortInfo:
    """Port information."""

    device: str
    description: str
    vid: int | None
    pid: int | None
    manufacturer: str | None
    serial_number: str | None
    location: str | None


def list_ports_info() -> dict[str, PortInfo]:
    """Get available serial ports with detailed params.

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
    timeout: float = 0.1,
    test_case: TestCase | None = None,
) -> bool:
    """Test connection to port.

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
                output = ser.readline()
                logger.debug(f"Serial output: {output}")

                if len(output) != len(test_case.expected_bytes):
                    logger.info(
                        f"Output length {len(output)} != "
                        f"expected length {len(test_case.expected_bytes)}"
                    )
                    return False

                if test_case.check_digit is not None:
                    return all(
                        output[idx] == test_case.expected_bytes[idx]
                        for idx in test_case.check_digit
                    )

                return output == test_case.expected_bytes
            return True
    except serial.SerialException as e:
        logger.error(f"Serial connection error: {e}")
        return False


def detect_baudrate(port: str, test_case: TestCase) -> int | None:
    """Find working baudrate for port.

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
    """Detect port with working baudrate.

    :param baudrate: Communication speed in baud
    :param test_case: Test case to verify communication
    :return: Port device name if found, None otherwise
    """
    for port in list_ports.comports():
        if test_connection(port.device, baudrate, test_case.timeout, test_case):
            return port.device
    return None


def scan_ports(test_case: TestCase) -> dict[str, int]:
    """Scan ports for working baudrate.

    :param test_case: Test case to verify communication
    :return: Dictionary mapping port device names to baudrates
    """
    results = {}
    # Disable logging during progress bar display
    for port in list_ports.comports():
        for baudrate in tqdm(BAUDRATES, desc=f"Testing {port.device}", leave=False):
            if test_connection(port.device, baudrate, test_case.timeout, test_case):
                logger.info(f"Found working baudrate: {baudrate} for {port.device}")
                results[port.device] = baudrate
                break
        if port.device not in results:
            logger.debug(f"Failed to connect to {port.device} at any baudrate")
            results[port.device] = 0

    return results


def print_ports() -> None:
    """List port names."""
    print(f"Available ports: {[port.device for port in list_ports.comports()]}")


def print_ports_info() -> None:
    """Print port information."""
    ports_info = list_ports_info()
    print("-" * 125)
    print(
        f"{'Port':<10} {'Description':<30} {'VID':<5} {'PID':<5} {'Manufacturer':<25} "
        f"{'Serial Number':<25} {'Location':<25}"
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
            f"{port:<10} {description:<30} {vid:<5} {pid:<5} {manufacturer:<25} "
            f"{serial_number:<25} {location:<25}"
        )
    print("-" * 125)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Serial port utilities")

    # Create mutually exclusive group
    group = parser.add_mutually_exclusive_group()
    group.add_argument("-l", "--list", action="store_true", help="List available ports")
    group.add_argument("-i", "--info", action="store_true", help="Show detailed port info")
    group.add_argument(
        "-t",
        "--test",
        action="store_true",
        help="Test connection at specified port and baudrate",
    )
    group.add_argument(
        "-db",
        "--detect-baudrate",
        action="store_true",
        help="Detect baudrate of a specific port",
    )
    group.add_argument(
        "-dp",
        "--detect-port",
        action="store_true",
        help="Detect ports that passes the test case",
    )
    group.add_argument(
        "-s", "--scan", action="store_true", help="Scan all ports using the test case"
    )

    parser.add_argument(
        "--test-case",
        "-tc",
        type=str,
        required=False,
        default=str(Path(__file__).parent / "default_serial_test_case.yaml"),
        help="Path to YAML file containing test case parameters",
    )
    parser.add_argument("--port", "-p", type=str, help="Serial port to use")
    parser.add_argument("--baudrate", "-b", type=int, help="Baudrate to use")
    parser.add_argument(
        "--log-level",
        "-ll",
        type=str,
        choices=["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"],
        default="WARNING",
        help="Set logging level",
    )
    args = parser.parse_args()

    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )

    if not any(
        [
            args.list,
            args.info,
            args.test,
            args.scan,
            args.detect_baudrate,
            args.detect_port,
        ]
    ):
        parser.print_help()
        exit(0)

    test_case = TestCase.from_str(args.test_case)

    if args.list:
        print_ports()

    if args.info:
        print_ports_info()

    if args.test:
        if not (args.port and args.baudrate):
            parser.error(
                "--test requires a port and baudrate to be specified with --port and --baudrate"
            )
        result = test_connection(
            args.port,
            baudrate=args.baudrate,
            test_case=test_case,
        )
        print(f"{result=}")

    if args.detect_baudrate:
        if not args.port:
            parser.error("--detect-baudrate requires a port to be specified with --port")
        baudrate = detect_baudrate(args.port, test_case)
        print(f"{baudrate=}")

    if args.detect_port:
        if not args.baudrate:
            parser.error("--detect-port requires a baudrate to be specified with --baudrate")
        port = detect_port(args.baudrate, test_case)
        print(f"{port=}")

    if args.scan:
        scan_results = scan_ports(test_case)
        print(f"{scan_results=}")
