### 1. Control Commands

1. **Motor Enable Control (0xF3)**

```
Command: Address + 0xF3 + 0xAB + Enable_Status + Sync_Flag + Checksum
Example: 01 F3 AB 01 00 6B
Input:
- Enable_Status: 01 (Enable) or 00 (Disable)
- Sync_Flag: 00 (No sync) or 01 (Sync)
Return:
- Success: 01 F3 02 6B
- Condition Not Met: 01 F3 E2 6B (motor stalled)
- Error: 01 00 EE 6B
```

2. **Speed Mode Control (0xF6)**

```
Command: Address + 0xF6 + Direction + Speed + Acceleration + Sync_Flag + Checksum
Example: 01 F6 01 05 DC 0A 00 6B
Input:
- Direction: 00 (CW) or 01 (CCW)
- Speed: 16-bit value (e.g., 05 DC = 1500 RPM)
- Acceleration: 8-bit value (0-255)
  Note: acc=0 means no acceleration curve
  Time calculation: t2-t1 = (256-acc)*50μs
- Sync_Flag: 00 or 01
Return:
- Success: 01 F6 02 6B
- Condition Not Met: 01 F6 E2 6B (stall protection or motor not enabled)
- Error: 01 00 EE 6B
```

3. **Position Mode Control (0xFD)**

```
Command: Address + 0xFD + Direction + Speed + Acceleration + Pulse_Count + Mode_Flag + Sync_Flag + Checksum
Example: 01 FD 01 05 DC 00 00 00 7D 00 00 00 6B
Input:
- Direction: 00 (CW) or 01 (CCW)
- Speed: 16-bit value
- Acceleration: 8-bit value
- Pulse_Count: 32-bit value (e.g., 00 00 7D 00 = 32000 pulses)
  Note: 3200 pulses = 1 revolution at 16 subdivisions
- Mode_Flag: 00 (Relative) or 01 (Absolute)
- Sync_Flag: 00 or 01
Return:
- Success: 01 FD 02 6B
- Condition Not Met: 01 FD E2 6B (stall protection or motor not enabled)
- Error: 01 00 EE 6B
```

4. **Emergency Stop (0xFE)**

```
Command: Address + 0xFE + 0x98 + Sync_Flag + Checksum
Example: 01 FE 98 00 6B
Input:
- Sync_Flag: 00 or 01
Return:
- Success: 01 FE 02 6B
- Condition Not Met: 01 FE E2 6B (stall protection or motor not enabled)
- Error: 01 00 EE 6B
```

5. **Multi-Machine Sync Movement (0xFF)**

```
Command: Address + 0xFF + 0x66 + Checksum
Example: 01 FF 66 6B
Return:
- Success: 01 FF 02 6B
- Condition Not Met: 01 FF E2 6B (stall protection or motor not enabled)
- Error: 01 00 EE 6B
```

### 2. Homing Commands

1. **Set Zero Position (0x93)**

```
Command: Address + 0x93 + 0x88 + Store_Flag + Checksum
Example: 01 93 88 01 6B
Input:
- Store_Flag: 00 (Don't store) or 01 (Store)
Return:
- Success: 01 93 02 6B
- Error: 01 00 EE 6B
```

2. **Trigger Homing (0x9A)**

```
Command: Address + 0x9A + Homing_Mode + Sync_Flag + Checksum
Example: 01 9A 00 00 6B
Input:
- Homing_Mode:
  00: Single-turn nearest homing
  01: Single-turn directional homing
  02: Multi-turn unlimited collision homing
  03: Multi-turn with limit switch homing
- Sync_Flag: 00 or 01
Return:
- Success: 01 9A 02 6B
- Condition Not Met: 01 9A E2 6B (stall protection, motor not enabled, or invalid zero position)
- Error: 01 00 EE 6B
```

3. **Abort Homing (0x9C)**

```
Command: Address + 0x9C + 0x48 + Checksum
Example: 01 9C 48 6B
Return:
- Success: 01 9C 02 6B
- Condition Not Met: 01 9C E2 6B (no homing operation in progress)
- Error: 01 00 EE 6B
```

### 3. Action Commands

1. **Trigger Encoder Calibration (0x06)**

```
Command: Address + 0x06 + 0x45 + Checksum
Example: 01 06 45 6B
Return:
- Success: 01 06 02 6B
- Condition Not Met: 01 06 E2 6B (open loop mode or stall protection)
- Error: 01 00 EE 6B
```

2. **Clear Current Position (0x0A)**

```
Command: Address + 0x0A + 0x6D + Checksum
Example: 01 0A 6D 6B
Return:
- Success: 01 0A 02 6B
- Error: 01 00 EE 6B
```

3. **Clear Stall Protection (0x0E)**

```
Command: Address + 0x0E + 0x52 + Checksum
Example: 01 0E 52 6B
Return:
- Success: 01 0E 02 6B
- Condition Not Met: 01 0E E2 6B (no stall protection triggered)
- Error: 01 00 EE 6B
```

4. **Factory Reset (0x0F)**

```
Command: Address + 0x0F + 0x5F + Checksum
Example: 01 0F 5F 6B
Return:
- Success: 01 0F 02 6B
- Error: 01 00 EE 6B
Note: Requires power cycle and encoder recalibration after reset
```

### 4. Read Commands

1. **Read Firmware Version (0x1F)**

```
Command: Address + 0x1F + Checksum
Example: 01 1F 6B
Return:
- Success: 01 1F [Firmware_Version] [Hardware_Version] 6B
  Example: 01 1F 7D 6F 6B
  Firmware: 0xF4 = Emm42_V5.0.0
  Hardware: 0x78 = ZDT_X42_V1.2
- Error: 01 00 EE 6B
```

2. **Read Phase Parameters (0x20)**

```
Command: Address + 0x20 + Checksum
Example: 01 20 6B
Return:
- Success: 01 20 [Resistance] [Inductance] 6B
  Example: 01 20 04 7A 0D 28 6B
  Resistance: 0x047A = 1146mΩ
  Inductance: 0x0D28 = 3368μH
- Error: 01 00 EE 6B
```

3. **Read Position PID Parameters (0x21)**

```
Command: Address + 0x21 + Checksum
Example: 01 21 6B
Return:
- Success: 01 21 [Kp] [Ki] [Kd] 6B
  Example: 01 21 00 00 F2 30 00 00 00 64 00 00 F2 30 6B
  Kp: 0x0000F230 = 62000
  Ki: 0x00000064 = 100
  Kd: 0x0000F230 = 62000
- Error: 01 00 EE 6B
```

4. **Read Bus Voltage (0x24)**

```
Command: Address + 0x24 + Checksum
Example: 01 24 6B
Return:
- Success: 01 24 [Voltage] 6B
  Example: 01 24 5C 6A 6B
  Voltage: 0x5C6A = 23658mV
- Error: 01 00 EE 6B
```

5. **Read Phase Current (0x27)**

```
Command: Address + 0x27 + Checksum
Example: 01 27 6B
Return:
- Success: 01 27 [Current] 6B
  Example: 01 27 02 73 6B
  Current: 0x0273 = 627mA
- Error: 01 00 EE 6B
```

6. **Read Calibrated Encoder Value (0x31)**

```
Command: Address + 0x31 + Checksum
Example: 01 31 6B
Return:
- Success: 01 31 [Encoder_Value] 6B
  Example: 01 31 8D 9E 6B
  Value: 0x8D9E = 36254 (0-65535 per revolution)
- Error: 01 00 EE 6B
```

7. **Read Input Pulse Count (0x32)**

```
Command: Address + 0x32 + Checksum
Example: 01 32 6B
Return:
- Success: 01 32 [Sign] [Pulse_Count] 6B
  Example: 01 32 01 00 00 0C 80 6B
  Sign: 00 (positive) or 01 (negative)
  Count: 32-bit value
- Error: 01 00 EE 6B
```

8. **Read Target Position (0x33)**

```
Command: Address + 0x33 + Checksum
Example: 01 33 6B
Return:
- Success: 01 33 [Sign] [Position] 6B
  Example: 01 33 01 00 01 00 00 6B
  Sign: 00 (positive) or 01 (negative)
  Position: 32-bit value (0-65535 per revolution)
- Error: 01 00 EE 6B
```

### 4. Read Commands (continued)

9. **Read Real-time Target Position (0x34)**

```
Command: Address + 0x34 + Checksum
Example: 01 34 6B
Return:
- Success: 01 34 [Sign] [Position] 6B
  Example: 01 34 01 00 01 00 00 6B
  Sign: 00 (positive) or 01 (negative)
  Position: 32-bit value (0-65535 per revolution)
- Error: 01 00 EE 6B
```

10. **Read Real-time Speed (0x35)**

```
Command: Address + 0x35 + Checksum
Example: 01 35 6B
Return:
- Success: 01 35 [Sign] [Speed] 6B
  Example: 01 35 01 05 DC 6B
  Sign: 00 (positive) or 01 (negative)
  Speed: 16-bit value in RPM
- Error: 01 00 EE 6B
```

11. **Read Real-time Position (0x36)**

```
Command: Address + 0x36 + Checksum
Example: 01 36 6B
Return:
- Success: 01 36 [Sign] [Position] 6B
  Example: 01 36 01 00 01 00 00 6B
  Sign: 00 (positive) or 01 (negative)
  Position: 32-bit value (0-65535 per revolution)
- Error: 01 00 EE 6B
```

12. **Read Position Error (0x37)**

```
Command: Address + 0x37 + Checksum
Example: 01 37 6B
Return:
- Success: 01 37 [Sign] [Error] 6B
  Example: 01 37 01 00 00 00 08 6B
  Sign: 00 (positive) or 01 (negative)
  Error: 32-bit value
- Error: 01 00 EE 6B
```

13. **Read Motor Status (0x3A)**

```
Command: Address + 0x3A + Checksum
Example: 01 3A 6B
Return:
- Success: 01 3A [Status] 6B
  Status bits:
  - 0x01: Motor enabled
  - 0x02: Position reached
  - 0x04: Motor stall
  - 0x08: Stall protection
- Error: 01 00 EE 6B
```

14. **Read Homing Status (0x3B)**

```
Command: Address + 0x3B + Checksum
Example: 01 3B 6B
Return:
- Success: 01 3B [Status] 6B
  Status bits:
  - 0x01: Encoder ready
  - 0x02: Calibration table ready
  - 0x04: Homing in progress
  - 0x08: Homing failed
- Error: 01 00 EE 6B
```

15. **Read Drive Configuration (0x42)**

```
Command Format:
Send (4 bytes):
  - Address (1 byte)
  - Command: 0x42 (1 byte)
  - SubCommand: 0x6C (1 byte)
  - Checksum: 0x6B (1 byte)
Example: 01 42 6C 6B

Response Format (35 bytes):
- Success: <Address:1><0x42:1><Data:33><Checksum:1>
- Error: 01 00 EE 6B

Response Data Structure (33 bytes):

1. Byte Count (1 byte)
   - Fixed value: 0x21 (33 decimal)

2. Parameter Count (1 byte)
   - Fixed value: 0x15 (21 decimal)

3. Motor Type (1 byte)
   - 0x19 (25): 1.8° motor
   - 0x32 (50): 0.9° motor

4. Pulse Control Mode (1 byte)
   - Values per screen options
   - Example: PUL_FOC = FOC vector closed-loop mode

5. Communication Mode (1 byte)
   - Values per screen options
   - Example: UART_FUN = UART/RS232/RS485

6. EN Pin Level (1 byte)
   - Hold = Always active

7. DIR Pin Direction (1 byte)
   - CW = Clockwise
   - CCW = Counter-clockwise

8. Subdivision (1 byte)
   - 0x00: 256 subdivisions
   - Other values: direct subdivision number
   Example: 0x10 = 16 subdivisions

9. Subdivision Interpolation (1 byte)
   - 0x00: Disable
   - 0x01: Enable

10. Auto Screen Off (1 byte)
    - 0x00: Disable
    - 0x01: Enable

11. Open Loop Current (2 bytes)
    Example: 0x03E8 = 1000mA

12. Max Closed Loop Current (2 bytes)
    Example: 0x0BB8 = 3000mA

13. Max Output Voltage (2 bytes)
    Example: 0x0FA0 = 4000mV

14. UART Baud Rate (1 byte)
    - 0x00: 9600
    - 0x01: 19200
    - 0x02: 38400
    - 0x03: 57600
    - 0x04: 115200

15. CAN Communication Rate (1 byte)
    - 0x00: 10000
    - 0x01: 20000
    - 0x02: 50000
    - 0x03: 100000
    - 0x04: 125000
    - 0x05: 250000
    - 0x06: 500000
    - 0x07: 1000000

16. Device ID (1 byte)
    Range: 0x01-0xFF

17. Communication Checksum (1 byte)
    Fixed: 0x6B

18. Command Response Mode (1 byte)
    - 0x00: Full response
    - 0x01: Acknowledge only

19. Stall Protection (1 byte)
    - 0x00: Disable
    - 0x01: Enable

20. Stall Parameters (6 bytes total)
    - Speed Threshold (2 bytes): Example: 0x0028 = 40 RPM
    - Current Threshold (2 bytes): Example: 0x0960 = 2400mA
    - Detection Time (2 bytes): Example: 0x0FA0 = 4000ms

21. Position Arrival Window (1 byte)
    Example: 0x01 = 0.1°

Example Response:
01 42 21 15 19 02 01 02 00 10 01 00 03 E8 0B B8 0F A0 04 06 01 6B 01 01 00 28 09 60 0F A0 01 6B

Notes:
- All multi-byte values are in big-endian format
- Total response length is fixed at 35 bytes
- Parameters must be read in exact order
- Stall protection triggers when ALL conditions are met:
  * Actual speed < Speed threshold AND
  * Actual current > Current threshold AND
  * Condition persists > Detection time
- Position arrival triggers when |Target - Actual| < Window value
```

16. **Read System Status (0x43)**

```
Command: Address + 0x43 + 0x7A + Checksum
Example: 01 43 7A 6B
Return:
- Success: 01 43 [System_Parameters] 6B
  Parameters include:
  - Bus voltage
  - Phase current
  - Encoder value
  - Position data
  - Speed data
  - Status flags
- Error: 01 00 EE 6B
```

### 5. Configuration Commands

1. **Modify Subdivision (0x84)**

```
Command: Address + 0x84 + 0x8A + Store_Flag + Subdivision + Checksum
Example: 01 84 8A 01 07 6B
Input:
- Store_Flag: 00 or 01
- Subdivision: 00 (256) or 01-FF
Return:
- Success: 01 84 02 6B
- Error: 01 00 EE 6B
```

2. **Modify ID Address (0xAE)**

```
Command: Address + 0xAE + 0x4B + Store_Flag + New_ID + Checksum
Example: 01 AE 4B 01 10 6B
Input:
- Store_Flag: 00 or 01
- New_ID: 8-bit value
Return:
- Success: 01 AE 02 6B
- Error: 01 00 EE 6B
```

3. **Switch Open/Closed Loop Mode (0x46)**

```
Command: Address + 0x46 + 0x69 + Store_Flag + Mode + Checksum
Example: 01 46 69 01 01 6B
Input:
- Store_Flag: 00 or 01
- Mode: 01 (Open loop) or 02 (Closed loop)
Return:
- Success: 01 46 02 6B
- Error: 01 00 EE 6B
```

4. **Modify Open Loop Current (0x44)**

```
Command: Address + 0x44 + 0x33 + Store_Flag + Current + Checksum
Example: 01 44 33 00 03 E8 6B
Input:
- Store_Flag: 00 or 01
- Current: 16-bit value in mA
Return:
- Success: 01 44 02 6B
- Error: 01 00 EE 6B
```

5. **Modify Position PID Parameters (0x4A)**

```
Command: Address + 0x4A + 0xC3 + Store_Flag + Kp + Ki + Kd + Checksum
Example: 01 4A C3 00 00 00 F2 30 00 00 00 64 00 00 F2 30 6B
Input:
- Store_Flag: 00 or 01
- Kp: 32-bit value
- Ki: 32-bit value
- Kd: 32-bit value
Return:
- Success: 01 4A 02 6B
- Error: 01 00 EE 6B
```

6. **Store Speed Mode Parameters (0xF7)**

```
Command: Address + 0xF7 + 0x1C + Store_Flag + Direction + Speed + Acceleration + EN_Control + Checksum
Example: 01 F7 1C 01 01 05 DC 0A 01 6B
Input:
- Store_Flag: 01
- Direction: 00 (CW) or 01 (CCW)
- Speed: 16-bit value in RPM
- Acceleration: 8-bit value
- EN_Control: 00 (disabled) or 01 (enabled)
Return:
- Success: 01 F7 02 6B
- Error: 01 00 EE 6B
```

7. **Modify Speed Input Scale (0x4F)**

```
Command: Address + 0x4F + 0x71 + Store_Flag + Scale_Flag + Checksum
Example: 01 4F 71 01 01 6B
Input:
- Store_Flag: 00 or 01
- Scale_Flag: 00 (normal) or 01 (divide by 10)
Return:
- Success: 01 4F 02 6B
- Error: 01 00 EE 6B
```

Common Notes:

- All values are in big-endian format
- Address is 01 by default but can be changed between 1 and F
- Checksum is typically 6B
- Store_Flag: 00 (temporary) or 01 (save to memory)
- Most command responses include condition checks for:
  - Motor enabled status
  - Stall protection status
  - Current operation status
- Position values are typically 32-bit where 65536 = one revolution
- Angle conversion: (Value \* 360) / 65536 degrees
