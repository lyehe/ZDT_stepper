## Command List

### Move Commands

enable
Format: [6 bytes] Addr + 0xF3 + 0xAB + Enable_Status + Sync + Checksum
Return: [4 bytes] Addr + 0xF3 + Status + Checksum
Errors: - Condition_Err: [4 bytes] Addr + 0xF3 + 0xE2 + Checksum - Error: [4 bytes] Addr + 0x00 + 0xEE + Checksum

jog
Format: [8 bytes] Addr + 0xF6 + Direction + Speed(2) + Acceleration + Sync + Checksum
Return: [4 bytes] Addr + 0xF6 + Status + Checksum
Errors: - Condition_Err: [4 bytes] Addr + 0xF6 + 0xE2 + Checksum - Error: [4 bytes] Addr + 0x00 + 0xEE + Checksum

move
Format: [13 bytes] Addr + 0xFD + Direction + Speed(2) + Acceleration + Pulse_Count(4) + Relative/Absolute_Mode + Sync + Checksum
Return: [4 bytes] Addr + 0xFD + Status + Checksum
Errors: - Condition_Err: [4 bytes] Addr + 0xFD + 0xE2 + Checksum - Error: [4 bytes] Addr + 0x00 + 0xEE + Checksum

estop
Format: [5 bytes] Addr + 0xFE + 0x98 + Sync + Checksum
Return: [4 bytes] Addr + 0xFE + Status + Checksum
Errors: - Condition_Err: [4 bytes] Addr + 0xFE + 0xE2 + Checksum - Error: [4 bytes] Addr + 0x00 + 0xEE + Checksum

sync_move
Format: [4 bytes] Addr + 0xFF + 0x66 + Checksum
Return: [4 bytes] Addr + 0xFF + Status + Checksum
Errors: - Condition_Err: [4 bytes] Addr + 0xFF + 0xE2 + Checksum - Error: [4 bytes] Addr + 0x00 + 0xEE + Checksum

### Home Commands

set_home
Format: [5 bytes] Addr + 0x93 + 0x88 + Store + Checksum
Return: [4 bytes] Addr + 0x93 + Status + Checksum
Errors: - Error: [4 bytes] Addr + 0x00 + 0xEE + Checksum

home
Format: [5 bytes] Addr + 0x9A + Homing_Mode + Sync + Checksum
Return: [4 bytes] Addr + 0x9A + Status + Checksum
Errors: - Condition_Err: [4 bytes] Addr + 0x9A + 0xE2 + Checksum - Error: [4 bytes] Addr + 0x00 + 0xEE + Checksum

stop_home
Format: [4 bytes] Addr + 0x9C + 0x48 + Checksum
Return: [4 bytes] Addr + 0x9C + Status + Checksum
Errors: - Condition_Err: [4 bytes] Addr + 0x9C + 0xE2 + Checksum - Error: [4 bytes] Addr + 0x00 + 0xEE + Checksum

get_home_param
Format: [3 bytes] Addr + 0x22 + Checksum
Return: [18 bytes] Addr + 0x22 + Homing_Mode + Homing_Direction + Homing_Speed(2) + Homing_Timeout(4) + Collision_Detection_Speed(2) + Collision_Detection_Current(2) + Collision_Detection_Time(2) + Auto_Home + Checksum
Errors: - Error: [4 bytes] Addr + 0x00 + 0xEE + Checksum

set_home_param
Format: [20 bytes] Addr + 0x4C + 0xAE + Store + Homing_Mode + Homing_Direction + Homing_Speed(2) + Homing_Timeout(4) + Collision_Detection_Speed(2) + Collision_Detection_Current(2) + Collision_Detection_Time(2) + Auto_Home + Checksum
Return: [4 bytes] Addr + 0x4C + Status + Checksum
Errors: - Error: [4 bytes] Addr + 0x00 + 0xEE + Checksum

get_home_status
Format: [3 bytes] Addr + 0x3B + Checksum
Return: [4 bytes] Addr + 0x3B + Homing_Status + Checksum
Errors: - Error: [4 bytes] Addr + 0x00 + 0xEE + Checksum

### System Commands

cal_encoder
Format: [4 bytes] Addr + 0x06 + 0x45 + Checksum
Return: [4 bytes] Addr + 0x06 + Status + Checksum
Errors: - Condition_Err: [4 bytes] Addr + 0x06 + 0xE2 + Checksum - Error: [4 bytes] Addr + 0x00 + 0xEE + Checksum

clear_pos
Format: [4 bytes] Addr + 0x0A + 0x6D + Checksum
Return: [4 bytes] Addr + 0x0A + Status + Checksum
Errors: - Error: [4 bytes] Addr + 0x00 + 0xEE + Checksum

clear_stall
Format: [4 bytes] Addr + 0x0E + 0x52 + Checksum
Return: [4 bytes] Addr + 0x0E + Status + Checksum
Errors: - Condition_Err: [4 bytes] Addr + 0x0E + 0xE2 + Checksum - Error: [4 bytes] Addr + 0x00 + 0xEE + Checksum

reset
Format: [4 bytes] Addr + 0x0F + 0x5F + Checksum
Return: [4 bytes] Addr + 0x0F + Status + Checksum
Errors: - Error: [4 bytes] Addr + 0x00 + 0xEE + Checksum

### Get Commands

get_version
Format: [3 bytes] Addr + 0x1F + Checksum
Return: [5 bytes] Addr + 0x1F + Firmware_Version + Hardware_Version + Checksum
Errors: - Error: [4 bytes] Addr + 0x00 + 0xEE + Checksum

get_motor_param
Format: [3 bytes] Addr + 0x20 + Checksum
Return: [7 bytes] Addr + 0x20 + Phase_Resistance(2) + Phase_Inductance(2) + Checksum
Errors: - Error: [4 bytes] Addr + 0x00 + 0xEE + Checksum

get_pid
Format: [3 bytes] Addr + 0x21 + Checksum
Return: [15 bytes] Addr + 0x21 + Kp(4) + Ki(4) + Kd(4) + Checksum
Errors: - Error: [4 bytes] Addr + 0x00 + 0xEE + Checksum

get_voltage
Format: [3 bytes] Addr + 0x24 + Checksum
Return: [5 bytes] Addr + 0x24 + Bus_Voltage(2) + Checksum
Errors: - Error: [4 bytes] Addr + 0x00 + 0xEE + Checksum

get_current
Format: [3 bytes] Addr + 0x27 + Checksum
Return: [5 bytes] Addr + 0x27 + Bus_Phase_Current(2) + Checksum
Errors: - Error: [4 bytes] Addr + 0x00 + 0xEE + Checksum

get_encoder
Format: [3 bytes] Addr + 0x31 + Checksum
Return: [5 bytes] Addr + 0x31 + Calibrated_Encoder_Value(2) + Checksum
Errors: - Error: [4 bytes] Addr + 0x00 + 0xEE + Checksum

get_pulse
Format: [3 bytes] Addr + 0x32 + Checksum
Return: [8 bytes] Addr + 0x32 + Sign + Input_Pulse_Count(4) + Checksum
Errors: - Error: [4 bytes] Addr + 0x00 + 0xEE + Checksum

get_target
Format: [3 bytes] Addr + 0x33 + Checksum
Return: [8 bytes] Addr + 0x33 + Sign + Motor_Target_Position(4) + Checksum
Errors: - Error: [4 bytes] Addr + 0x00 + 0xEE + Checksum

get_setpoint
Format: [3 bytes] Addr + 0x34 + Checksum
Return: [8 bytes] Addr + 0x34 + Sign + Motor_Target_Position(4) + Checksum
Errors: - Error: [4 bytes] Addr + 0x00 + 0xEE + Checksum

get_speed
Format: [3 bytes] Addr + 0x35 + Checksum
Return: [6 bytes] Addr + 0x35 + Sign + Motor_Real_Time_Speed(2) + Checksum
Errors: - Error: [4 bytes] Addr + 0x00 + 0xEE + Checksum

get_pos
Format: [3 bytes] Addr + 0x36 + Checksum
Return: [8 bytes] Addr + 0x36 + Sign + Motor_Real_Time_Position(4) + Checksum
Errors: - Error: [4 bytes] Addr + 0x00 + 0xEE + Checksum

get_error
Format: [3 bytes] Addr + 0x37 + Checksum
Return: [8 bytes] Addr + 0x37 + Sign + Motor_Position_Error(4) + Checksum
Errors: - Error: [4 bytes] Addr + 0x00 + 0xEE + Checksum

get_status
Format: [3 bytes] Addr + 0x3A + Checksum
Return: [4 bytes] Addr + 0x3A + Status + Checksum
Errors: - Error: [4 bytes] Addr + 0x00 + 0xEE + Checksum

get_config
Format: [4 bytes] Addr + 0x42 + 0x6C + Checksum
Return: [33 bytes] Addr + 0x42 + 0x21 + 0x15 + Motor_Type + Control_Mode + Comm_Mode + En_Level + Dir_Level + Microsteps + Microstep_Interp + Screen_Off + Open_Loop_Current(2) + Closed_Loop_Current(2) + Max_Voltage(2) + Baud_Rate + CAN_Rate + ID_Addr + Checksum_mode + Response_Mode + Stall_Protect + Stall_Speed(2) + Stall_Current(2) + Stall_Time(2) + Pos_Window(2) + Checksum
Errors: - Error: [4 bytes] Addr + 0x00 + 0xEE + Checksum

get_sys_status
Format: [4 bytes] Addr + 0x43 + 0x7A + Checksum
Return: [31 bytes] Addr + 0x43 + 0x1F + 0x09 + Bus_Voltage(2) + Bus_Phase_Current(2) + Calibrated_Encoder_Value(2) + Sign + Motor_Target_Position(4) + Sign + Motor_Real_Time_Speed(2) + Sign + Motor_Real_Time_Position(4) + Sign + Motor_Position_Error(4) + Ready_Status + Motor_Status + Checksum
Errors: - Error: [4 bytes] Addr + 0x00 + 0xEE + Checksum

set_microstep
Format: [6 bytes] Addr + 0x84 + 0x8A + Store + Microstep_Value + Checksum
Return: [4 bytes] Addr + 0x84 + Status + Checksum
Errors: - Error: [4 bytes] Addr + 0x00 + 0xEE + Checksum

set_id
Format: [6 bytes] Addr + 0xAE + 0x4B + Store + ID_Addr + Checksum
Return: [4 bytes] Addr + 0xAE + Status + Checksum
Errors: - Error: [4 bytes] Addr + 0x00 + 0xEE + Checksum

set_mode
Format: [6 bytes] Addr + 0x46 + 0x69 + Store + Control_Mode + Checksum
Return: [4 bytes] Addr + 0x46 + Status + Checksum
Errors: - Error: [4 bytes] Addr + 0x00 + 0xEE + Checksum

set_current
Format: [7 bytes] Addr + 0x44 + 0x33 + Store + Open_Loop_Current(2) + Checksum
Return: [4 bytes] Addr + 0x44 + Status + Checksum
Errors: - Error: [4 bytes] Addr + 0x00 + 0xEE + Checksum

set_config
Format: [33 bytes] Addr + 0x48 + 0xD1 + Store + Motor_Type + Control_Mode + Comm_Mode + En_Level + Dir_Level + Microsteps + Microstep_Interp + Screen_Off + Open_Loop_Current(2) + Closed_Loop_Current(2) + Max_Voltage(2) + Baud_Rate + CAN_Rate + ID_Addr + Verify_Mode + Response_Mode + Stall_Protect + Stall_Speed(2) + Stall_Current(2) + Stall_Time(2) + Pos_Window(2) + Checksum
Return: [4 bytes] Addr + 0x48 + Status + Checksum
Errors: - Error: [4 bytes] Addr + 0x00 + 0xEE + Checksum

set_pid
Format: [16 bytes] Addr + 0x4A + 0xC3 + Store + Kp(4) + Ki(4) + Kd(4) + Checksum
Return: [4 bytes] Addr + 0x4A + Status + Checksum
Errors: - Error: [4 bytes] Addr + 0x00 + 0xEE + Checksum

save_speed
Format: [9 bytes] Addr + 0xF7 + 0x1C + Store + Direction + Speed(2) + Acceleration + En_Control + Checksum
Return: [4 bytes] Addr + 0xF7 + Status + Checksum
Errors: - Error: [4 bytes] Addr + 0x00 + 0xEE + Checksum

set_reduction
Format: [6 bytes] Addr + 0x4F + 0x71 + Store + Speed_Reduction + Checksum
Return: [4 bytes] Addr + 0x4F + Status + Checksum
Errors: - Error: [4 bytes] Addr + 0x00 + 0xEE + Checksum
