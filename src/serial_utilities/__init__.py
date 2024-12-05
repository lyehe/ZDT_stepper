"""Serial utilities for testing and detecting connections."""

from .serial_utilities import (
    BAUDRATES,
    PathVar,
    PortInfo,
    TestCase,
    detect_baudrate,
    detect_port,
    list_ports_info,
    print_ports,
    print_ports_info,
    scan_ports,
    test_connection,
)

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
