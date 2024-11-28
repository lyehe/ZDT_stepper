from stepper.stepper_constants import CHECKSUM_TYPE, PROTOCOL


def _calculate_xor_checksum(data: bytes) -> int:
    """Calculate XOR checksum of bytes"""
    checksum = 0
    for byte in data:
        checksum ^= byte
    return checksum


def _calculate_crc8(data: bytes) -> int:
    """Calculate CRC-8 checksum of bytes
    Using CRC-8 with polynomial x^8 + x^2 + x + 1 (0x07)
    """
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ 0x07
            else:
                crc <<= 1
        crc &= 0xFF
    return crc


def calculate_checksum(checksum_type: CHECKSUM_TYPE, command_bytes: bytes) -> int:
    """Calculate checksum based on selected method"""
    match checksum_type:
        case CHECKSUM_TYPE.FIXED:
            return PROTOCOL.CHECKSUM_BYTE
        case CHECKSUM_TYPE.XOR:
            return _calculate_xor_checksum(command_bytes)
        case CHECKSUM_TYPE.CRC8:
            return _calculate_crc8(command_bytes)
        case _:
            raise ValueError(f"Invalid checksum type: {checksum_type}")
