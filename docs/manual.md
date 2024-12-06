### Communication Control List

#### Common Parameters

Store:
    0x00: Not stored in EEPROM
    0x01: Store in EEPROM

Sign:
    0x00: Positive 
    0x01: Negative

Sync:
    0x00: Multi-machine sync disabled
    0x01: Multi-machine sync enabled

Direction:
    0x00: CW rotation
    0x01: CCW rotation

Motor Status:
    Possible values: 0x00 to 0x0F (16 values)

    Value conditions:
    0x00: Motor disabled, not in position, not stalled, no protection
    0x01: Motor enabled only
    0x02: In position only
    0x03: Motor enabled and in position
    0x04: Stalled only
    0x05: Motor enabled and stalled
    0x06: In position and stalled
    0x07: Motor enabled, in position, and stalled
    0x08: Protection triggered only
    0x09: Motor enabled with protection triggered
    0x0A: In position with protection triggered
    0x0B: Motor enabled, in position, with protection triggered
    0x0C: Stalled with protection triggered
    0x0D: Motor enabled, stalled, with protection triggered
    0x0E: In position, stalled, with protection triggered
    0x0F: All conditions active (enabled, in position, stalled, protection)

Ready Status:
    Possible values: 0x00 to 0x0F (16 values)

    Value conditions:
    0x00: Nothing ready, no homing in progress or failed
    0x01: Encoder ready only
    0x02: Calibration table ready only
    0x03: Encoder and calibration table ready
    0x04: Homing in progress only
    0x05: Encoder ready with homing in progress
    0x06: Calibration ready with homing in progress
    0x07: Encoder and calibration ready with homing in progress
    0x08: Homing failed only
    0x09: Encoder ready with homing failed
    0x0A: Calibration ready with homing failed
    0x0B: Encoder and calibration ready with homing failed
    0x0C: Homing in progress and failed
    0x0D: Encoder ready, homing in progress and failed
    0x0E: Calibration ready, homing in progress and failed
    0x0F: All conditions active (all ready, homing in progress and failed)

Error return: 
    0x01 0x00 0xEE 0x6B

#### Control Action List

enable
Format: Addr + 0xF3 + 0xAB + Enable_Status + Sync + Checksum 
Return: Addr + 0xF3 + Status + Checksum 
Example:
    Send: 01 F3 AB 01 00 6B
    Correct return: 01 F3 02 6B
    Condition_Err return: 01 F3 E2 6B
Explanation:
    Enable_Status:
        0x01: Enable motor
        0x00: Disable motor

jog
Format: Addr + 0xF6 + Direction + Speed(2) + Acceleration + Sync + Checksum 
Return: Addr + 0xF6 + Status + Checksum 
Example:
    Send: 01 F6 01 05 DC 0A 00 6B
    Correct return: 01 F6 02 6B
    Condition_Err return: 01 F6 E2 6B
Explanation:
    Speed:
        2 bytes, unit: RPM
        Example: 0x05DC = 1500 RPM
    Acceleration:
        1 byte, range: 0-255
        Example: 0x0A = acceleration level 10
Conditions not met:
    - Stall protection triggered
    - Motor not enabled
Note:
    - Acceleration level 0 indicates not using curve acceleration/deceleration, runs directly at set speed
    - Curve acceleration/deceleration time calculation formula: t2 - t1 = (256 - acc) * 50(us), Vt2 = Vt1 + 1(RPM)

move
Format: Addr + 0xFD + Direction + Speed(2) + Acceleration + Pulse_Count(4) + Relative/Absolute_Mode + Sync + Checksum 
Return: Addr + 0xFD + Status + Checksum 
Example:
    Send: 01 FD 01 05 DC 00 00 00 7D 00 00 00 6B
    Correct return: 01 FD 02 6B
    Condition_Err return: 01 FD E2 6B
Explanation:
    Speed:
        2 bytes, unit: RPM
        Example: 0x05DC = 1500 RPM
    Acceleration:
        1 byte, range: 0-255
        Example: 0x00 = acceleration level 0
    Pulse_Count:
        4 bytes
        Example: 0x00007D00 = 32000 pulses
    Relative/Absolute_Mode:
        0x00: Relative position mode
        0x01: Absolute position mode
Conditions not met:
    - Stall protection triggered
    - Motor not enabled
Note:
    - Under 16 microsteps, sending 3200 pulses rotates motor one revolution
    - Acceleration level 0 indicates not using curve acceleration/deceleration, runs directly at set speed
    - Curve acceleration/deceleration time calculation formula: t2 - t1 = (256 - acc) * 50(us), Vt2 = Vt1 + 1(RPM)

estop
Format: Addr + 0xFE + 0x98 + Sync + Checksum 
Return: Addr + 0xFE + Status + Checksum 
Example:
    Send: 01 FE 98 00 6B
    Correct return: 01 FE 02 6B
    Condition_Err return: 01 FE E2 6B
Conditions not met:
    - Stall protection triggered
    - Motor not enabled
Note:
    - Makes motor stop immediately (emergency brake)

sync_move
Format: Addr + 0xFF + 0x66 + Checksum 
Return: Addr + 0xFF + Status + Checksum 
Example:
    Send: 01 FF 66 6B
    Correct return: 01 FF 02 6B
    Condition_Err return: 01 FF E2 6B
Conditions not met:
    - Stall protection triggered
    - Motor not enabled
Note:
    - Makes multiple motors start movement synchronously

#### Home Return List

set_home
Format: Addr + 0x93 + 0x88 + Store + Checksum 
Return: Addr + 0x93 + Status + Checksum 
Example:
    Send: 01 93 88 01 6B
    Correct return: 01 93 02 6B
Conditions not met:
    - Stall protection triggered
    - Motor not enabled
Note:
    - Can let motor turn to desired position, then send this to set single-turn home position

home
Format: Addr + 0x9A + Homing_Mode + Sync + Checksum 
Return: Addr + 0x9A + Status + Checksum 
Example:
    Send: 01 9A 00 00 6B
    Correct return: 01 9A 02 6B
    Condition_Err return: 01 9A E2 6B
Explanation:
    Homing_Mode:
        0x00: Trigger single-turn nearest return
        0x01: Trigger single-turn directional return
        0x02: Trigger multi-turn unlimited collision return
        0x03: Trigger multi-turn with limit switch return
Conditions not met:
    - Stall protection triggered
    - Motor not enabled
    - Single-turn home position value invalid

stop_home
Format: Addr + 0x9C + 0x48 + Checksum 
Return: Addr + 0x9C + Status + Checksum 
Example:
    Send: 01 9C 48 6B
    Correct return: 01 9C 02 6B
    Condition_Err return: 01 9C E2 6B
Explanation:
    During homing process, can use this to force interrupt and exit homing operation
Conditions not met:
    No homing operation currently triggered

get_home_param
Format: Addr + 0x22 + Checksum 
Return: Addr + 0x22 + Homing_Mode + Homing_Direction + Homing_Speed(2) + Homing_Timeout(4) + Collision_Detection_Speed(2) + Collision_Detection_Current(2) + Collision_Detection_Time(2) + Auto_Home + Checksum
Example:
    Send: 01 22 6B
    Correct return: 01 22 00 00 00 1E 00 00 27 10 01 2C 03 20 00 3C 00 6B
Explanation:
    Homing_Mode:
        0x00: Single-turn nearest return
    Homing_Direction:
        0x00: CW
        0x01: CCW
    Homing_Speed:
        2 bytes, unit: RPM
        Example: 0x001E = 30 RPM
    Homing_Timeout:
        4 bytes, unit: ms
        Example: 0x00002710 = 10000 ms
    Collision_Detection_Speed:
        2 bytes, unit: RPM
        Example: 0x012C = 300 RPM
    Collision_Detection_Current:
        2 bytes, unit: mA
        Example: 0x0320 = 800 mA
    Collision_Detection_Time:
        2 bytes, unit: ms
        Example: 0x003C = 60 ms
    Auto_Home:
        0x00: Disabled
        0x01: Enabled

set_home_param
Format: Addr + 0x4C + 0xAE + Store + Homing_Mode + Homing_Direction + Homing_Speed(2) + Homing_Timeout(4) + Collision_Detection_Speed(2) + Collision_Detection_Current(2) + Collision_Detection_Time(2) + Auto_Home + Checksum
Return: Addr + 0x4C + Status + Checksum
Example:
    Send: 01 4C AE 01 00 00 00 1E 00 00 27 10 01 2C 03 20 00 3C 00 6B
    Correct return: 01 4C 02 6B
Explanation:
    Homing_Mode:
        0x00: Single-turn nearest return
        0x01: Single-turn directional return
        0x02: Multi-turn unlimited collision return
        0x03: Multi-turn with limit switch return
    Homing_Direction:
        0x00: CW
        0x01: CCW
    Homing_Speed:
        2 bytes, unit: RPM
        Example: 0x001E = 30 RPM
    Homing_Timeout:
        4 bytes, unit: ms
        Example: 0x00002710 = 10000 ms
    Collision_Detection_Speed:
        2 bytes, unit: RPM
        Example: 0x012C = 300 RPM
    Collision_Detection_Current:
        2 bytes, unit: mA
        Example: 0x0320 = 800 mA
    Collision_Detection_Time:
        2 bytes, unit: ms
        Example: 0x003C = 60 ms
    Auto_Home:
        0x00: Disabled
        0x01: Enabled

get_home_status
Format: Addr + 0x3B + Checksum
Return: Addr + 0x3B + Homing_Status + Checksum
Example:
    Send: 01 3B 6B
    Correct return: 01 3B 03 6B
Explanation:
    Homing_Status:
        Encoder Ready Status:
            0x01: Ready
            0x00: Not ready
        Calibration Table Ready Status:
            0x02: Ready
            0x00: Not ready
        Homing in Progress:
            0x04: In progress
            0x00: Not in progress
        Homing Failed:
            0x08: Failed
            0x00: Not failed

#### Trigger Action List

cal_encoder
Format: Addr + 0x06 + 0x45 + Checksum
Return: Addr + 0x06 + Status + Checksum
Example:
    Send: 01 06 45 6B
    Correct return: 01 06 02 6B
    Condition_Err return: 01 06 E2 6B 
Explanation:
    Trigger encoder calibration, corresponding to "Cal" menu on screen
Conditions not met:
    - Current is open loop mode
    - Stall protection triggered

clear_pos
Format: Addr + 0x0A + 0x6D + Checksum
Return: Addr + 0x0A + Status + Checksum
Example:
    Send: 01 0A 6D 6B
    Correct return: 01 0A 02 6B
Explanation:
    Clear current position angle, position error, pulse count etc. all to zero

clear_stall
Format: Addr + 0x0E + 0x52 + Checksum
Return: Addr + 0x0E + Status + Checksum
Example:
    Send: 01 0E 52 6B
    Correct return: 01 0E 02 6B
    Condition_Err return: 01 0E E2 6B
Explanation:
    After motor stalls, sending this can clear stall protection
Conditions not met:
    - Stall protection not triggered

reset
Format: Addr + 0x0F + 0x5F + Checksum
Return: Addr + 0x0F + Status + Checksum
Example:
    Send: 01 0F 5F 6B
    Correct return: 01 0F 02 6B
Explanation:
    Sending this can restore factory settings
Note:
    - After factory reset needs to power cycle and recalibrate encoder with no load
    - After triggering factory reset, blue light turns on

get_version
Format: Addr + 0x1F + Checksum
Return: Addr + 0x1F + Firmware_Version + Hardware_Version + Checksum
Example:
    Send: 01 1F 6B
    Correct return: 01 1F 7D 6F 6B
Explanation:
    Firmware_Version:
        0xF4: Emm42_V5.0.0
    Hardware_Version:
        0x78: ZDT_X42_V1.2 version

get_motor_param
Format: Addr + 0x20 + Checksum
Return: Addr + 0x20 + Phase_Resistance(2) + Phase_Inductance(2) + Checksum
Example:
    Send: 01 20 6B
    Correct return: 01 20 04 7A 0D 28 6B
Explanation:
    Phase_Resistance:
        2 bytes, unit: mΩ
        Example: 0x047A = 1146 mΩ
    Phase_Inductance:
        2 bytes, unit: μH
        Example: 0x0D28 = 3368 μH

get_pid
Format: Addr + 0x21 + Checksum
Return: Addr + 0x21 + Kp(4) + Ki(4) + Kd(4) + Checksum
Example:
    Send: 01 21 6B
    Correct return: 01 21 00 00 F2 30 00 00 00 64 00 00 F2 30 6B
Explanation:
    Kp:
        4 bytes
        Example: 0x0000F230 = 62000
    Ki:
        4 bytes
        Example: 0x00000064 = 100
    Kd:
        4 bytes
        Example: 0x0000F230 = 62000

get_voltage
Format: Addr + 0x24 + Checksum
Return: Addr + 0x24 + Bus_Voltage + Checksum
Example:
    Send: 01 24 6B
    Correct return: 01 24 5C 6A 6B
Explanation:
    Bus_Voltage:
        2 bytes, unit: mV
        Example: 0x5C6A = 23658 mV
Note:
    Input voltage will have voltage drop after reverse protection diode

get_current
Format: Addr + 0x27 + Checksum
Return: Addr + 0x27 + Bus_Phase_Current + Checksum
Example:
    Send: 01 27 6B
    Correct return: 01 27 02 73 6B
Explanation:
    Bus_Phase_Current:
        2 bytes, unit: mA
        Example: 0x0273 = 627 mA

get_encoder
Format: Addr + 0x31 + Checksum
Return: Addr + 0x31 + Calibrated_Encoder_Value + Checksum
Example:
    Send: 01 31 6B
    Correct return: 01 31 8D 9E 6B
Explanation:
    Calibrated_Encoder_Value:
        2 bytes
        Example: 0x8D9E = 36254
Note:
    After linearization calibration, encoder value is quadrupled internally, one revolution value range is 0-65535

get_pulse
Format: Addr + 0x32 + Checksum
Return: Addr + 0x32 + Sign + Input_Pulse_Count(4) + Checksum
Example:
    Send: 01 32 6B
    Correct return: 01 32 01 00 00 0C 80 6B
Explanation:
    Input_Pulse_Count:
        1+4 bytes
        Example: 0x00000C80 = -3200 pulses (with Sign = 0x01)

get_target
Format: Addr + 0x33 + Checksum
Return: Addr + 0x33 + Sign + Motor_Target_Position(4) + Checksum
Example:
    Send: 01 33 6B
    Correct return: 01 33 01 00 01 00 00 6B
Explanation:
    Motor_Target_Position:
        1+4 bytes, range: 0-65535 per revolution
        Example: 0x00010000 = -360.0° (with Sign = 0x01)

get_setpoint 
Format: Addr + 0x34 + Checksum
Return: Addr + 0x34 + Sign + Motor_Target_Position(4) + Checksum
Example:
    Send: 01 34 6B
    Correct return: 01 34 01 00 01 00 00 6B
Explanation:
    Motor_Target_Position:
        1+4 bytes, range: 0-65535 per revolution
        Example: 0x00010000 = -360.0° (with Sign = 0x01)

get_speed
Format: Addr + 0x35 + Checksum
Return: Addr + 0x35 + Sign + Motor_Real_Time_Speed(2) + Checksum
Example:
    Send: 01 35 6B
    Correct return: 01 35 01 05 DC 6B
Explanation:
    Motor_Real_Time_Speed:
        1+2 bytes, unit: RPM
        Example: 0x05DC = -1500 RPM (with Sign = 0x01)

get_pos
Format: Addr + 0x36 + Checksum
Return: Addr + 0x36 + Sign + Motor_Real_Time_Position(4) + Checksum
Example:
    Send: 01 36 6B
    Correct return: 01 36 01 00 01 00 00 6B
Explanation:
    Motor_Real_Time_Position:
        1+4 bytes, range: 0-65535 per revolution
        Example: 0x00010000 = -360.0° (with Sign = 0x01)

get_error
Format: Addr + 0x37 + Checksum 
Return: Addr + 0x37 + Sign + Motor_Position_Error(4) + Checksum 
Example:
    Send: 01 37 6B
    Correct return: 01 37 01 00 00 00 08 6B
Explanation:
    Motor_Position_Error:
        1+4 bytes, range: 0-65535 per revolution
        Example: 0x00000008 = -0.0439453125° (with Sign = 0x01)

get_status
Format: Addr + 0x3A + Checksum 
Return: Addr + 0x3A + Status + Checksum 
Example:
    Send: 01 3A 6B
    Correct return: 01 3A 03 6B

get_config
Format: Addr + 0x42 + 0x6C + Checksum 
Return: Addr + 0x42 + 0x21 + 0x15 + Motor_Type + Control_Mode + Comm_Mode + En_Level + Dir_Level + Microsteps + Microstep_Interp + Screen_Off + Open_Loop_Current(2) + Closed_Loop_Current(2) + Max_Voltage(2) + Baud_Rate + CAN_Rate + ID_Addr + Verify_Mode + Response_Mode + Stall_Protect + Stall_Speed(2) + Stall_Current(2) + Stall_Time(2) + Pos_Window(2) + Checksum
Example:
    Send: 01 42 6C 6B
    Correct return: 01 42 21 15 19 02 02 02 00 10 01 00 03 E8 0B B8 0F A0 05 07 01 00 01 01 00 28 09 60 0F A0 00 01 6B
Explanation:
    Motor_Type:
        0x19: 1.8° motor
        0x32: 0.9° motor
    Control_Mode:
        0x02: PUL_FOC (FOC vector closed loop mode)
    Comm_Mode:
        0x02: UART_FUN (UART/RS232/RS485)
    En_Level:
        0x02: Hold (always valid)
    Dir_Level:
        0x00: CW direction
    Microsteps:
        0x00: 256 microsteps
        0x01-0xFF: 1-255 microsteps
    Microstep_Interp:
        0x00: Disabled
        0x01: Enabled
    Screen_Off:
        0x00: Disabled
        0x01: Enabled
    Open_Loop_Current:
        2 bytes, unit: mA
        Example: 0x03E8 = 1000 mA
    Closed_Loop_Current:
        2 bytes, unit: mA
        Example: 0x0BB8 = 3000 mA
    Max_Voltage:
        2 bytes, unit: mV
        Example: 0x0FA0 = 4000 mV
    Baud_Rate:
        0x05: 115200
    CAN_Rate:
        0x07: 500000
    ID_Addr:
        0x01: Allow broadcast address batch modification
    Verify_Mode:
        0x00: 0x6B verification
    Response_Mode:
        0x01: Receive (only reply command received)
    Stall_Protect:
        0x00: Disabled
        0x01: Enabled
    Stall_Speed:
        2 bytes, unit: RPM
        Example: 0x0028 = 40 RPM
    Stall_Current:
        2 bytes, unit: mA
        Example: 0x0960 = 2400 mA
    Stall_Time:
        2 bytes, unit: ms
        Example: 0x0FA0 = 4000 ms
    Pos_Window:
        2 bytes, unit: 0.1°
        Example: 0x0001 = 0.1°


get_sys_status
Format: Addr + 0x43 + 0x7A + Checksum
Return: Addr + 0x43 + 0x1F + 0x09 + Bus_Voltage(2) + Bus_Phase_Current(2) + Calibrated_Encoder_Value(2) + Sign + Motor_Target_Position(4) + Sign + Motor_Real_Time_Speed(2) + Sign + Motor_Real_Time_Position(4) + Sign + Motor_Position_Error(4) + Ready_Status + Motor_Status + Checksum
Example:
    Send: 01 43 7A 6B
    Correct return: 01 43 1F 09 5C 67 00 03 43 EB 01 00 01 00 00 00 00 00 01 00 01 00 00 01 00 00 00 08 03 03 6B
Explanation:
    Bus_Voltage:
        2 bytes, unit: mV
        Example: 0x5C67 = 23655 mV
    Bus_Phase_Current:
        2 bytes, unit: mA
        Example: 0x0003 = 3 mA
    Calibrated_Encoder_Value:
        2 bytes
        Example: 0x43EB = 17387
    Motor_Target_Position:
        1+4 bytes
        Example: 0x00010000 = -360° (with Sign = 0x01)
        Formula: -(value * 360)/65536
    Motor_Real_Time_Speed:
        1+2 bytes, unit: RPM
        Example: 0x0000 = 0 RPM (with Sign = 0x00)
    Motor_Real_Time_Position:
        1+4 bytes
        Example: 0x00010000 = -360° (with Sign = 0x01)
        Formula: -(value * 360)/65536
    Motor_Position_Error:
        1+4 bytes
        Example: 0x00000001 = -0.044° (with Sign = 0x01)
        Formula: -(value * 8 * 360)/65536

#### Modify Parameter List

set_microstep
Format: Addr + 0x84 + 0x8A + Store + Microstep_Value + Checksum 
Return: Addr + 0x84 + Status + Checksum 
Example:
    Send: 01 84 8A 01 07 6B
    Correct return: 01 84 02 6B
Explanation:
    Microstep_Value:
        0x00: 256 microsteps
        0x01-0x08: 1-8 microsteps

set_id
Format: Addr + 0xAE + 0x4B + Store + ID_Addr + Checksum 
Return: Addr + 0xAE + Status + Checksum 
Example:
    Send: 01 AE 4B 01 10 6B
    Correct return: 01 AE 02 6B
Explanation:
    ID_Addr:
        1 byte, range: 0x01-0xFF
        Example: 0x10 = ID 16

set_mode
Format: Addr + 0x46 + 0x69 + Store + Control_Mode + Checksum 
Return: Addr + 0x46 + Status + Checksum 
Example:
    Send: 01 46 69 01 01 6B
    Correct return: 01 46 02 6B
Explanation:
    Control_Mode:
        0x01: Open loop mode
        0x02: Closed loop mode

set_current
Format: Addr + 0x44 + 0x33 + Store + Open_Loop_Current(2) + Checksum 
Return: Addr + 0x44 + Status + Checksum 
Example:
    Send: 01 44 33 00 03 E8 6B
    Correct return: 01 44 02 6B
Explanation:
    Open_Loop_Current:
        2 bytes, unit: mA
        Example: 0x03E8 = 1000 mA

set_config
Format: Addr + 0x48 + 0xD1 + Store + Motor_Type + Control_Mode + Comm_Mode + En_Level + Dir_Level + Microsteps + Microstep_Interp + Screen_Off + Open_Loop_Current(2) + Closed_Loop_Current(2) + Max_Voltage(2) + Baud_Rate + CAN_Rate + ID_Addr + Verify_Mode + Response_Mode + Stall_Protect + Stall_Speed(2) + Stall_Current(2) + Stall_Time(2) + Pos_Window(2) + Checksum
Return: Addr + 0x48 + Status + Checksum 
Example:
    Send: 01 48 D1 01 19 02 02 02 00 10 01 00 03 E8 0B B8 0F A0 05 07 01 00 01 01 00 28 09 60 0F A0 00 01 6B
    Correct return: 01 48 02 6B
Explanation:
    Motor_Type:
        0x19: 1.8° motor
        0x32: 0.9° motor
    Control_Mode:
        0x02: PUL_FOC (FOC vector closed loop mode)
    Comm_Mode:
        0x02: UART_FUN (UART/RS232/RS485)
    En_Level:
        0x02: Hold (always valid)
    Dir_Level:
        0x00: CW direction
    Microsteps:
        0x00: 256 microsteps
        0x01-0xFF: 1-255 microsteps
    Microstep_Interp:
        0x00: Disabled
        0x01: Enabled
    Screen_Off:
        0x00: Disabled
        0x01: Enabled
    Open_Loop_Current:
        2 bytes, unit: mA
        Example: 0x03E8 = 1000 mA
    Closed_Loop_Current:
        2 bytes, unit: mA
        Example: 0x0BB8 = 3000 mA
    Max_Voltage:
        2 bytes, unit: mV
        Example: 0x0FA0 = 4000 mV
    Baud_Rate:
        0x05: 115200
    CAN_Rate:
        0x07: 500000
    ID_Addr:
        0x01: Allow broadcast address batch modification
    Verify_Mode:
        0x00: 0x6B verification
    Response_Mode:
        0x01: Receive (only reply command received)
    Stall_Protect:
        0x00: Disabled
        0x01: Enabled
    Stall_Speed:
        2 bytes, unit: RPM
        Example: 0x0028 = 40 RPM
    Stall_Current:
        2 bytes, unit: mA
        Example: 0x0960 = 2400 mA
    Stall_Time:
        2 bytes, unit: ms
        Example: 0x0FA0 = 4000 ms
    Pos_Window:
        2 bytes, unit: 0.1°
        Example: 0x0001 = 0.1°

set_pid
Format: Addr + 0x4A + 0xC3 + Store + Kp(4) + Ki(4) + Kd(4) + Checksum 
Return: Addr + 0x4A + Status + Checksum 
Example:
    Send: 01 4A C3 00 00 00 F2 30 00 00 00 64 00 00 F2 30 6B
    Correct return: 01 4A 02 6B
Explanation:
    Kp:
        4 bytes
        Example: 0x0000F230 = 62000
    Ki:
        4 bytes
        Example: 0x00000064 = 100
    Kd:
        4 bytes
        Example: 0x0000F230 = 62000

save_speed
Format: Addr + 0xF7 + 0x1C + Store + Direction + Speed(2) + Acceleration + En_Control + Checksum 
Return: Addr + 0xF7 + Status + Checksum 
Example:
    Send: 01 F7 1C 01 01 05 DC 0A 01 6B
    Correct return: 01 F7 02 6B
Explanation:
    Speed:
        2 bytes, unit: RPM
        Example: 0x05DC = 1500 RPM
    Acceleration:
        1 byte, range: 0-255
        Example: 0x0A = acceleration level 10
    En_Control:
        0x00: Disable En pin control
        0x01: Enable En pin control
Note:
    - After sending, each power-up motor will automatically accelerate to set speed with set acceleration
    - En pin control motor start valid level can be selected in menu En
    - Select H for high level start, low level stop

set_reduction
Format: Addr + 0x4F + 0x71 + Store + Speed_Reduction + Checksum 
Return: Addr + 0x4F + Status + Checksum 
Example:
    Send: 01 4F 71 01 01 6B
    Correct return: 01 4F 02 6B
Explanation:
    Speed_Reduction:
        0x00: Disabled
        0x01: Enabled
Note:
    - After enabled, communication control sent speed will be reduced by 10x
    - For example if sending 1RPM speed, motor will run at 0.1RPM
    - Affects speed mode control, position mode control, and stored speed parameters
