## 介绍
## 快速开始
这是一个通过 UART 串行连接控制张大头步进电机控制器的 Python 库。它包含了规格说明书中所有已记录的命令，并以类型安全的方式处理所有参数。它还可以扩展附加功能、CAN 通信，并可以修改用于其他串行控制器设备。经过完整测试，兼容 Python 3.10 及以上版本。

### 安装
```bash
pip install zdt_stepper
```

### 基础示例
```python
from serial import Serial
from stepper.device import Device
from stepper.stepper_core.parameters import DeviceParams
from stepper.stepper_core.configs import Address

# 连接电机
serial = Serial("COM4", 115200, timeout=0.1)
device = Device(
    device_params=DeviceParams(
        serial_connection=serial,
        address=Address(0x01)
    )
)

# 基本运动控制
device.enable()
device.move_cw(1000)  # 顺时针移动1000脉冲
device.move_to(0)     # 移动到绝对位置0
device.jog(500)       # 以500脉冲/秒的速度连续运动
device.stop()         # 停止运动
```

### 特性
- 全面的电机控制（启用/禁用、绝对/相对运动、点动）
- 带加速度控制的高级运动配置
- 实时位置和状态监控
- PID 调整和闭环控制
- 配置管理（细分、电流、保护设置）
- 回零和校准程序
- 简单控制的命令字符串接口
- 广泛的错误处理和调试

### 状态监控
```python
# 获取各种状态信息
print(f"位置：{device.real_time_position.position}")
print(f"速度：{device.real_time_speed.speed}")
print(f"状态：已启用={device.is_enabled}, 到位={device.is_in_position}")
print(f"电压：{device.bus_voltage.voltage}")
print(f"电流：{device.phase_current.current}")
```

### 配置
```python
# 配置电机参数
device.set_microstep(32)
device.set_open_loop_current(1000)
device.set_pid(p=100, i=50, d=20)
device.set_speed(1000)
device.set_acceleration(500)

# 永久保存设置
device.enable_store()
device.set_config()
```

### 命令接口（开发中）
```python
# 使用简单的命令字符串进行控制
device.parse_cmd("HOM")          # 电机回零
device.parse_cmd("MOV 1000")     # 移动到位置1000
device.parse_cmd("JOG 500")      # 以500脉冲/秒的速度点动
device.parse_cmd("STP")          # 停止运动
device.parse_cmd("PID 100 50 20") # 设置PID参数
```

### 命令对象接口（用于调试）
```python
from serial import Serial
from stepper.commands.move import Enable, Move
from stepper.stepper_core.parameters import DeviceParams, PositionParams
from stepper.stepper_core.configs import (
    Address, Direction, Speed,
    Acceleration, PulseCount, AbsoluteFlag
)

# 连接电机
serial = Serial("COM4", 115200, timeout=0.1)
device = DeviceParams(
    serial_connection=serial,
    address=Address(0x01)
)

# 启用电机
Enable(device=device).status
# 配置运动
params = PositionParams(
    direction=Direction.CW,
    speed=Speed(500),
    acceleration=Acceleration(127),
    pulse_count=PulseCount(160),
    absolute=AbsoluteFlag.RELATIVE
)

# 移动电机
Move(device=device, params=params).status

# 移动到绝对位置
params.absolute = AbsoluteFlag.ABSOLUTE
Move(device=device, params=params).status
```
### 状态监控
```python
from stepper.commands.get import GetSysStatus

status = GetSysStatus(device=device).raw_data.data_dict
print(status)
```
### 配置
```python
from src.stepper.commands.set import SetConfig
from src.stepper.stepper_constants import Microstep, StallTime

config = GetConfig(device=device).raw_data
config.microstep = Microstep(value=32)
config.stall_time = StallTime(value=100)
SetConfig(device=device, params=config).status
```

### 开发
```bash
# 安装开发依赖
pip install zdt_stepper[dev]
```

### Device类下的所有功能列表

系统信息
- init_time（初始化时间）
- version（版本）
- motor_rh（电机运行时间）
- bus_voltage（总线电压）
- phase_current（相电流）
- encoder_value（编码器值）
- pulse_count（脉冲计数）
- target_position（目标位置）
- open_loop_setpoint（开环设定点）
- real_time_speed（实时速度）
- real_time_position（实时位置）
- position_error（位置误差）
- sys_status（系统状态）
- status（状态）

状态标志
- is_enabled（是否启用）
- is_in_position（是否到位）
- is_stalled（是否堵转）
- is_stall_protection_active（堵转保护是否激活）
- encoder_ready（编码器就绪）
- encoder_calibrated（编码器已校准）
- is_homing（是否在回零）
- is_homing_failed（回零是否失败）
- is_sync（是否同步）
- is_store（是否存储）

运动参数
- jog_direction（点动方向）
- jog_speed（点动速度）
- jog_acceleration（点动加速度）
- move_speed（移动速度）
- move_acceleration（移动加速度）
- move_direction（移动方向）
- move_pulse_count（移动脉冲计数）
- move_mode（移动模式）

配置
- pid（PID参数）
- config（配置）
- homing_params（回零参数）
- homing_status（回零状态）
- state_dict（状态字典）
- params_dict（参数字典）

PID控制
- set_pid()（设置PID）
- set_p()（设置P）
- set_i()（设置I）
- set_d()（设置D）

设备配置
- set_config()（设置配置）
- set_stepper_type()（设置步进电机类型）
- set_control_mode()（设置控制模式）
- set_communication_mode()（设置通信模式）
- set_enable_level()（设置使能电平）
- set_default_direction()（设置默认方向）
- set_microstep()（设置细分）
- set_microstep_interp()（设置细分插值）
- set_screen_off()（设置屏幕关闭）
- set_open_loop_current()（设置开环电流）
- set_max_closed_loop_current()（设置最大闭环电流）
- set_max_voltage()（设置最大电压）
- set_baud_rate()（设置波特率）
- set_canrate()（设置CAN速率）
- set_id()（设置ID）
- set_checksum_mode()（设置校验模式）
- set_response_mode()（设置响应模式）
- set_stall_protect()（设置堵转保护）
- set_stall_speed()（设置堵转速度）
- set_stall_current()（设置堵转电流）
- set_stall_time()（设置堵转时间）
- set_on_target_window()（设置到位窗口）

起始速度配置
- set_start_speed_params()（设置起始速度参数）
- set_start_direction()（设置起始方向）
- set_start_speed()（设置起始速度）
- set_start_acceleration()（设置起始加速度）
- set_start_en_control()（设置起始使能控制）

系统配置
- set_loop_mode()（设置闭环模式）
- set_speed_reduction()（设置减速比）

系统命令
- sys_calibrate_encoder()（系统校准编码器）
- sys_factory_reset()（系统恢复出厂设置）
- sys_clear_stall()（系统清除堵转）
- sys_zero_all_positions()（系统清零所有位置）

回零方法
- set_homing_params()（设置回零参数）
- set_homing_mode()（设置回零模式）
- set_homing_direction()（设置回零方向）
- set_homing_speed()（设置回零速度）
- set_homing_timeout()（设置回零超时）
- set_collision_detection_speed()（设置碰撞检测速度）
- set_collision_detection_current()（设置碰撞检测电流）
- set_collision_detection_time()（设置碰撞检测时间）
- set_auto_home()（设置自动回零）
- home()（回零）
- set_home()（设置原点）
- stop_home()（停止回零）

运动控制
- enable()（启用）
- disable()（禁用）
- estop()（紧急停止）
- jog()（点动）
- jog_cw()（顺时针点动）
- jog_ccw()（逆时针点动）
- jog_at_speed()（指定速度点动）
- set_jog_speed()（设置点动速度）
- set_jog_direction()（设置点动方向）
- set_jog_acceleration()（设置点动加速度）
- stop()（停止）
- move()（移动）
- move_to()（移动到）
- move_cw()（顺时针移动）
- move_ccw()（逆时针移动）
- set_move_speed()（设置移动速度）
- set_move_direction()（设置移动方向）
- set_move_acceleration()（设置移动加速度）
- set_speed()（设置速度）
- set_direction()（设置方向）
- set_acceleration()（设置加速度）
- sync_move()（同步移动）

同步和存储控制
- enable_sync()（启用同步）
- disable_sync()（禁用同步）
- enable_store()（启用存储）
- disable_store()（禁用存储）

实用方法
- wait()（等待）
- debug()（调试）
- resolve_bug()（解决bug）
- tic()（计时开始）
- toc()（计时结束）
- jog_time()（点动时间）
- parse_cmd()（解析命令）

## 许可证
MIT许可证
