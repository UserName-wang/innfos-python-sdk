"""
Constants for the INNFOS Python SDK
"""

class ErrorsDefine:
    """Error codes defined in the SDK"""
    ERR_NONE = 0                    # 无错误, 执行器正常工作
    ERR_ACTUATOR_OVERVOLTAGE = 0x01 # 执行器过压, 供电电压超过安全范围
    ERR_ACTUATOR_UNDERVOLTAGE = 0x02 # 执行器欠压, 供电电压低于安全范围
    ERR_ACTUATOR_LOCKED_ROTOR = 0x04 # 执行器转子锁定, 电机无法转动
    ERR_ACTUATOR_OVERHEATING = 0x08 # 执行器过热, 电机温度过高
    ERR_ACTUATOR_READ_OR_WRITE = 0x10 # 执行器读写错误, 通信读写失败
    ERR_ACTUATOR_MULTI_TURN = 0x20  # 执行器多圈计数错误, 编码器多圈计数异常
    ERR_INVERTOR_TEMPERATURE_SENSOR = 0x40 # 逆变器温度传感器错误, 温度传感器故障
    ERR_CAN_COMMUNICATION = 0x80    # CAN通信错误, CAN总线通信异常
    ERR_ACTUATOR_TEMPERATURE_SENSOR = 0x100 # 执行器温度传感器错误, 电机温度传感器故障
    ERR_STEP_OVER = 0x200           # 步进超差, 实际位置与指令位置偏差过大
    ERR_DRV_PROTECTION = 0x400      # 驱动保护, 驱动器进入保护状态
    ERR_CODER_DISABLED = 0x800      # 编码器禁用, 编码器未启用
    ERR_ACTUATOR_DISCONNECTION = 0x801 # 执行器断开连接, 无法与执行器通信
    ERR_CAN_DISCONNECTION = 0x802   # CAN断开连接, CAN通信中断
    ERR_IP_ADDRESS_NOT_FOUND = 0x803 # IP地址未找到, 无法解析目标IP地址
    ERR_ABNORMAL_SHUTDOWN = 0x804   # 异常关机, 执行器异常关闭
    ERR_SHUTDOWN_SAVING = 0x805     # 关机保存, 执行器正常关机保存参数
    ERR_IP_HAS_BIND = 0x806         # IP已绑定, IP地址已被其他设备占用
    ERR_ID_UNUNIQUE = 0x807         # ID不唯一, 网络中存在重复ID
    ERR_IP_CONFLICT = 0x808         # IP冲突, 网络中存在重复IP
    ERR_UNKNOWN = 0xffff            # 未知错误, 未定义的错误类型

class ControlMode:
    """Control modes for the actuators"""
    CURRENT_MODE = 0x01             # 电流控制模式, 直接控制电机电流输出
    SPEED_MODE = 0x02               # 速度控制模式, 控制电机转速
    POSITION_MODE = 0x03            # 位置控制模式, 控制电机目标位置
    TRAPEZOIDAL_POSITION_MODE = 0x06 # 梯形位置控制模式, 带轨迹规划的位置控制
    TRAPEZOIDAL_SPEED_MODE = 0x07    # 梯形速度控制模式, 带轨迹规划的速度控制
    HOMING_MODE = 0x08              # 回零控制模式, 执行器寻找零点位置

class Command:
    """Command codes for communication with actuators"""
    # Read commands
    HANDSHAKE = 0x44        # 与ECB握手, 建立通信连接
    READ_MODE = 0x55        # 查询执行器当前模式, 读取执行器所管理的执行器的当前的模式
    READ_VOLTAGE = 0x45     # 读取执行器电压值, 获取当前电压状态,（数据值为真实值的2^10倍）
    READ_CURRENT = 0x04     # 读取指定ID执行器的当前电流值，电流真实值需要乘以电流满量程，单位为A
    READ_SPEED = 0x05       # 读取执行器速度值, 获取当前速度状态
    READ_POSITION = 0x06    # 读取执行器位置值, 获取当前位置信息
    READ_TEMPERATURE = 0x5F # 读取执行器温度值, 获取当前电机温度
    
    # Write commands
    SET_MODE = 0x07         # 设置执行器模式, 更改执行器控制模式
    ENABLE_ACTUATOR = 0x2A  # 启用/禁用执行器, 控制执行器使能状态
    SET_CURRENT = 0x08      # 设置执行器电流, 控制执行器电流输出
    SET_SPEED = 0x09        # 设置执行器速度, 控制执行器转速
    SET_POSITION = 0x0A     # 设置执行器位置, 控制执行器目标位置
    STORE_PARAMETERS = 0x0D # 存储参数到EEPROM, 保存当前参数设置
    
    # Special commands
    QUERY_ACTUATORS = 0x02  # 查询执行器地址, 发现网络中的执行器
    CLEAR_ALARM = 0xFE      # 清除报警状态, 复位执行器报警信息

# Protocol constants
PROTOCOL = {
    'HEADER': 0xEE,        # 数据包头部标识, 所有数据包以0xEE开始
    'TAIL': 0xED,          # 数据包尾部标识, 所有数据包以0xED结束
    'BROADCAST_ID': 0x00,  # 广播ID, 发送给所有执行器的特殊ID
    'DEFAULT_PORT': 2000,  # 默认端口号, 与ECB通信的UDP端口
    'DEFAULT_IP': '192.168.1.30'  # 默认IP地址, ECB的默认IP地址
}