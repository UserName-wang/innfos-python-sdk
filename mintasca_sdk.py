import socket
import struct
import time
from CRC_mINTASCA import calculate_crc16

class MintascaActuator:
    def __init__(self, ecb_ip="192.168.1.30", port=2000, timeout=1.0, debug=False):
        """
        初始化Mintasca执行器通信模块
        
        Args:
            ecb_ip (str): ECB的IP地址，默认为192.168.1.30
            port (int): 通信端口，默认为2000
            timeout (float): 通信超时时间，默认为1.0秒
            debug (bool): 是否启用调试模式，默认为False
        """
        self.ecb_ip = ecb_ip
        self.port = port
        self.timeout = timeout
        self.debug = debug
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.settimeout(timeout)
        
        # 存储执行器信息，格式为 {actuator_id: {serial_number: bytes, mode: int, last_update: float}}
        self.actuators = {}
        
        # 标志位，用于避免重复清理
        self._cleaned_up = False
        
        # 握手并获取执行器信息
        self._handshake()
        self._discover_actuators()
        # 在查询到执行器ID之后，先将执行器使能，然后再查询其工作模式
        self._enable_all_actuators()
        # 将所有执行器的工作模式设置为位置模式
        self._set_all_actuators_to_position_mode()
        self._get_actuator_modes()
    
    def _calculate_crc(self, data):
        """
        计算CRC校验码（使用CRC16算法）
        根据抓包分析，需要使用Modbus CRC16算法
        """
        return calculate_crc16(data)
    
    def _build_packet(self, address, command, data=b'', no_crc=False):
        """
        构建通信数据包
        
        Args:
            address (int): 设备地址
            command (int): 指令符
            data (bytes): 数据内容
            no_crc (bool): 是否不添加CRC校验码
            
        Returns:
            bytes: 完整的数据包
        """
        # 数据长度
        data_length = len(data)
        
        # 构建包头
        packet = bytearray()
        packet.append(0xEE)  # 帧头
        packet.append(address)  # 设备地址
        packet.append(command)  # 指令符
        packet.extend(struct.pack('>H', data_length))  # 数据长度（大端）
        packet.extend(data)  # 数据内容
        
        # 根据参数决定是否添加CRC
        # 特定命令不需要CRC: 0x44 (握手)、0x02 (查询执行器)、0x0A (设置位置) 和 0x55 (查询执行器工作模式)
        if not no_crc and command not in [0x44, 0x02, 0x0A, 0x55]:
            # CRC校验只计算数据内容，即数据位数之后到crc校验之前的所有内容
            if len(data) > 0:
                # 如果有数据，则对数据内容计算CRC
                crc = self._calculate_crc(data)
                packet.extend(struct.pack('>H', crc))  # CRC校验码（大端序）
            else:
                # 如果没有数据，仍然需要计算CRC，但针对空数据计算
                crc = self._calculate_crc(b'')
                packet.extend(struct.pack('>H', crc))  # CRC校验码（大端序）
        packet.append(0xED)  # 帧尾
        
        if self.debug:
            print(f"构建数据包: {' '.join(f'{b:02X}' for b in packet)}")
        
        return bytes(packet)
    
    def _send_packet(self, packet):
        """
        发送数据包
        
        Args:
            packet (bytes): 要发送的数据包
        """
        if self.debug:
            print(f"发送数据包到 {self.ecb_ip}:{self.port}: {' '.join(f'{b:02X}' for b in packet)}")
        
        self.socket.sendto(packet, (self.ecb_ip, self.port))
    
    def _send_packet_and_receive(self, packet):
        """
        发送数据包并接收响应
        
        Args:
            packet (bytes): 要发送的数据包
            
        Returns:
            bytes: 接收到的响应数据
        """
        if self.debug:
            print(f"发送数据包到 {self.ecb_ip}:{self.port}: {' '.join(f'{b:02X}' for b in packet)}")
        
        self.socket.sendto(packet, (self.ecb_ip, self.port))
        try:
            response, addr = self.socket.recvfrom(1024)
            if self.debug:
                print(f"收到响应数据包: {' '.join(f'{b:02X}' for b in response)} (来自 {addr})")
            return response
        except socket.timeout:
            if self.debug:
                print("接收响应超时")
            raise Exception("接收响应超时")
    
    def _parse_response(self, response):
        """
        解析响应数据包
        
        Args:
            response (bytes): 接收到的响应数据
            
        Returns:
            dict: 解析后的数据
        """
        if len(response) < 7:
            raise Exception("响应数据包长度不足")
        
        # 检查帧头和帧尾
        if response[0] != 0xEE or response[-1] != 0xED:
            raise Exception("无效的数据包格式")
        
        # 解析包内容
        address = response[1]
        command = response[2]
        data_length = struct.unpack('>H', response[3:5])[0]
        
        # 根据数据长度提取数据内容和CRC
        data = b''
        crc = None
        
        # 特殊处理某些命令的响应
        if command in [0x44, 0x02]:
            # 这些命令的响应没有CRC
            if data_length > 0 and len(response) >= 6 + data_length:
                data = response[5:5+data_length]
        elif data_length > 0:
            # 数据 + CRC (2字节)
            # 计算数据结束位置：5(包头到数据长度) + data_length(数据长度)
            data_end_index = 5 + data_length
            # 检查响应是否包含完整的数据
            if len(response) >= data_end_index + 1:  # +1 for ED at the end
                data = response[5:data_end_index]
                # 检查是否有CRC（在数据之后，ED之前）
                if len(response) >= data_end_index + 2 + 1:  # +2 for CRC, +1 for ED
                    try:
                        crc = struct.unpack('<H', response[data_end_index:data_end_index+2])[0]
                    except:
                        crc = None
        else:
            # 尝试解析可能存在的CRC (2字节)
            if len(response) >= 7 + 2:
                try:
                    crc = struct.unpack('<H', response[5:7])[0]
                except:
                    crc = None
        
        result = {
            'address': address,
            'command': command,
            'data_length': data_length,
            'data': data,
            'crc': crc
        }
        
        if self.debug:
            crc_str = f"0x{crc:04X}" if crc is not None else "无"
            data_str = data.hex() if data else '无'
            print(f"解析响应数据包: 地址=0x{address:02X}, 指令=0x{command:02X}, "
                  f"数据长度={data_length}, 数据={data_str}, CRC={crc_str}")
        
        return result
    
    def _handshake(self):
        """
        与ECB进行握手
        发送指令: EE 00 44 00 00 ED (不带CRC)
        """
        if self.debug:
            print("开始与ECB握手...")
        
        # 构建握手数据包: EE 00 44 00 00 ED (不带CRC)
        handshake_packet = self._build_packet(0x00, 0x44, b'', no_crc=True)
        
        # 发送握手包并接收响应
        response = self._send_packet_and_receive(handshake_packet)
        
        # 解析响应
        parsed_response = self._parse_response(response)
        
        # 检查响应是否正确
        if parsed_response['address'] == 0x00 and parsed_response['command'] == 0x44:
            if self.debug:
                print("与ECB握手成功")
        else:
            raise Exception("与ECB握手失败")
    
    def _discover_actuators(self):
        """
        查询在线的执行器ID
        发送指令: EE 00 02 00 00 ED （只需要发送一次即可）
        通信成功会返回所有执行器的ID和序列号，由于要轮询查找执行器，
        本条指令大约需要0.5s左右才能返回执行器id，如果长时间收不到执行器地址信息，
        可认为没有执行器成功连接。
        返回指令实例: EE 06 02 00 04 01 64 5A DF 3B 3F ED
        其中06即为执行器的ID，01 64 5A DF为执行器序列号。
        (当有多个执行器成功连接后，会多次返回该指令)
        """
        if self.debug:
            print("开始查询在线执行器...")
        
        # 发送查询执行器指令: EE 00 02 00 00 ED (不带CRC)
        discover_packet = self._build_packet(0x00, 0x02, b'', no_crc=True)
        
        # 只发送一次查询指令
        self._send_packet(discover_packet)
        
        # 收集所有执行器响应，设置更长的超时时间以确保收集完整
        actuators_found = {}
        start_time = time.time()
        discovery_timeout = 2.5  # 增加超时时间到2.5秒以确保收集完整
        
        try:
            while time.time() - start_time < discovery_timeout:
                # 注意：这里不需要再次发送指令，只需要接收响应
                try:
                    self.socket.settimeout(0.05)  # 设置较短的超时时间
                    response, addr = self.socket.recvfrom(1024)
                    if self.debug:
                        print(f"收到响应数据包: {' '.join(f'{b:02X}' for b in response)} (来自 {addr})")
                    
                    # 手动解析响应，因为可能有多个响应需要处理
                    if len(response) >= 7 and response[0] == 0xEE and response[-1] == 0xED:
                        address = response[1]
                        command = response[2]
                        data_length = struct.unpack('>H', response[3:5])[0]
                        
                        # 检查是否为执行器ID信息响应 (指令符0x02)
                        if command == 0x02:
                            actuator_id = address  # 执行器ID是响应中的地址字段
                            # 避免重复添加相同的执行器
                            if actuator_id not in actuators_found:
                                # 检查数据长度是否足够
                                if data_length >= 1:
                                    # 数据从索引5开始
                                    data_end_index = 5 + data_length
                                    if len(response) >= data_end_index + 1:  # 确保数据和包尾都在
                                        data = response[5:data_end_index]
                                        # 根据协议，第一个字节是类型(0x02)，后面是序列号
                                        if len(data) >= 1 and data[0] == 0x02:
                                            # 提取序列号（最多4字节）
                                            serial_bytes = data[1:]  # 从索引1开始的所有数据都是序列号
                                            # 如果不足4字节，在末尾补0
                                            while len(serial_bytes) < 4:
                                                serial_bytes += b'\x00'
                                            serial_number = serial_bytes[:4]
                                                
                                            # 将序列号转换为十进制数
                                            serial_number_int = struct.unpack('>I', serial_number)[0]  # 大端序转换
                                            actuators_found[actuator_id] = {
                                                'id': actuator_id,
                                                'serial_number_hex': serial_number,  # 保留十六进制数据
                                                'serial_number_int': serial_number_int,  # 转换为十进制数
                                                'last_update': time.time()  # 添加更新时间戳
                                            }
                                            if self.debug:
                                                serial_str = serial_number.hex() if serial_number else '无'
                                                print(f"发现执行器 ID: 0x{actuator_id:02X}, 序列号: {serial_str} (十进制: {serial_number_int})")
                                        elif self.debug:
                                            print(f"执行器 ID: 0x{actuator_id:02X} 数据格式不正确，跳过 (数据长度: {len(data)}, 数据内容: {data.hex() if data else '无'})")
                                    elif self.debug:
                                        print(f"执行器 ID: 0x{actuator_id:02X} 响应数据不完整，跳过")
                                elif self.debug:
                                    print(f"执行器 ID: 0x{actuator_id:02X} 数据长度不足，跳过 (数据长度字段: {data_length})")
                            elif self.debug:
                                print(f"跳过已存在的执行器 ID: 0x{actuator_id:02X}")
                except socket.timeout:
                    # 超时是正常的，继续循环直到达到总的超时时间
                    pass
                    
        except Exception as e:
            # 其他异常处理
            if self.debug:
                print(f"查询执行器结束: {e}")
                import traceback
                traceback.print_exc()
        finally:
            # 恢复原始超时设置
            self.socket.settimeout(self.timeout)
        
        self.actuators = actuators_found
        if self.debug:
            print(f"共发现 {len(self.actuators)} 个执行器")
    
    def _get_mode_string(self, mode):
        """
        将模式值转换为可读的字符串
        
        Args:
            mode (int): 模式值
            
        Returns:
            str: 模式描述字符串
        """
        mode_map = {
            0x01: "电流模式",
            0x02: "速度模式",
            0x03: "位置模式",
            0x06: "梯形位置模式",
            0x07: "梯形速度模式",
            0x08: "homing模式"
        }
        return mode_map.get(mode, f"未知模式(0x{mode:02X})")
    
    def _get_actuator_mode(self, actuator_id):
        """
        获取指定执行器的工作模式
        
        Args:
            actuator_id (int): 执行器ID
            
        Returns:
            指令符	指令符
            01	电流模式
            02	速度模式
            03	位置模式
            06	梯形位置模式
            07	梯形速度模式
            08	homing模式
        """
        if self.debug:
            print(f"获取执行器 0x{actuator_id:02X} 的工作模式...")
        
        # 指令 0x55: 查询执行器当前模式
        status_packet = self._build_packet(actuator_id, 0x55, b'')
        
        try:
            # 发送状态查询包并接收响应
            response = self._send_packet_and_receive(status_packet)
            parsed_response = self._parse_response(response)
            
            # 检查响应是否是针对该执行器的状态响应
            if parsed_response['command'] == 0x55 and parsed_response['address'] == actuator_id:
                # 检查是否有数据并解析模式
                if parsed_response['data_length'] >= 1 and len(parsed_response['data']) >= 1:
                    mode = parsed_response['data'][0]
                    if self.debug:
                        mode_str = self._get_mode_string(mode)
                        print(f"执行器 0x{actuator_id:02X} 当前模式: {mode_str}")
                    return mode
                else:
                    if self.debug:
                        print(f"执行器 0x{actuator_id:02X} 模式读取失败：无数据")
                    return None
            else:
                # 如果收到的不是针对该执行器的状态响应，可能是其他执行器的响应
                if self.debug:
                    print(f"执行器 0x{actuator_id:02X} 模式读取收到非预期响应，指令={parsed_response['command']:02X}，地址={parsed_response['address']:02X}")
                return None
                
        except Exception as e:
            if self.debug:
                print(f"执行器 0x{actuator_id:02X} 模式读取异常: {e}")
            return None
    
    def _get_actuator_modes(self):
        """
        获取所有执行器的当前工作模式,查询指令:0x55,查询执行器当前模式,	读取执行器所管理的执行器的当前的模式。
        
        """
        if self.debug:
            print("开始获取执行器工作模式...")
        
        current_time = time.time()
        
        for actuator_id in list(self.actuators.keys()):  # 使用list()避免在迭代时修改字典
            # 为每个执行器设置一个较短的超时时间，避免长时间等待
            start_time = time.time()
            mode_obtained = False
            attempt_count = 0
            
            while time.time() - start_time < 2.0 and not mode_obtained and attempt_count < 5:  # 2秒超时，最多尝试5次
                try:
                    mode_packet = self._build_packet(actuator_id, 0x55, b'')
                    response = self._send_packet_and_receive(mode_packet)
                    parsed_response = self._parse_response(response)
                    
                    # 检查响应是否是针对模式查询的正确响应
                    if parsed_response['command'] == 0x55 and parsed_response['address'] == actuator_id:
                        if parsed_response['data_length'] >= 1 and len(parsed_response['data']) >= 1:
                            mode = parsed_response['data'][0]
                            self.actuators[actuator_id]['mode'] = mode
                            self.actuators[actuator_id]['last_update'] = current_time  # 更新最后更新时间
                            mode_str = self._get_mode_string(mode)
                            if self.debug:
                                print(f"执行器 0x{actuator_id:02X} 当前模式: {mode_str}")
                            mode_obtained = True
                        else:
                            # 保留旧模式信息，仅标记为过期
                            if 'mode' in self.actuators[actuator_id]:
                                if self.debug:
                                    mode_str = self._get_mode_string(self.actuators[actuator_id]['mode'])
                                    print(f"执行器 0x{actuator_id:02X} 模式读取失败：无数据，保留旧模式 {mode_str}")
                            else:
                                if self.debug:
                                    print(f"执行器 0x{actuator_id:02X} 模式读取失败：无数据，且无旧模式")
                            self.actuators[actuator_id]['last_update'] = current_time - 10  # 标记为过期
                            mode_obtained = True
                    else:
                        # 如果收到的不是针对该执行器的模式响应，继续等待正确的响应
                        if self.debug:
                            print(f"执行器 0x{actuator_id:02X} 模式查询：收到非预期响应，指令={parsed_response['command']:02X}，地址={parsed_response['address']:02X}")
                        # 等待一小段时间再重试
                        time.sleep(0.1)
                        attempt_count += 1
                    
                except Exception as e:
                    if self.debug:
                        print(f"执行器 0x{actuator_id:02X} 模式读取异常: {e}")
                    # 等待一小段时间再重试
                    time.sleep(0.1)
                    attempt_count += 1
            
            # 如果未能获取模式，且执行器信息已存在，则标记为过期
            if not mode_obtained and actuator_id in self.actuators:
                self.actuators[actuator_id]['last_update'] = current_time - 10  # 标记为过期
                if self.debug:
                    print(f"执行器 0x{actuator_id:02X} 未能获取模式信息，标记为过期")

    def get_actuator_info(self, actuator_id):
        """
        获取指定执行器的信息
        
        Args:
            actuator_id (int): 执行器ID
            
        Returns:
            dict: 包含执行器信息的字典，如果执行器不存在则返回None
        """
        return self.actuators.get(actuator_id)
    
    def get_all_actuators_info(self):
        """
        获取所有执行器的信息
        
        Returns:
            dict: 包含所有执行器信息的字典
        """
        return self.actuators.copy()
    
    def is_actuator_online(self, actuator_id, max_age=5.0):
        """
        检查执行器是否在线（即最近更新时间在max_age秒内）
        
        Args:
            actuator_id (int): 执行器ID
            max_age (float): 最大允许的更新时间间隔（秒）
            
        Returns:
            bool: 如果执行器在线返回True，否则返回False
        """
        actuator_info = self.actuators.get(actuator_id)
        if not actuator_info:
            return False
            
        last_update = actuator_info.get('last_update', 0)
        return time.time() - last_update <= max_age
    
    def refresh_actuator_info(self):
        """
        刷新执行器信息，重新发现所有在线执行器并获取它们的模式
        """
        actuators_before = self.actuators.copy()
        self.actuators = {}  # 清空现有执行器信息
        self._discover_actuators()
        self._get_actuator_modes()
        actuators_after = self.actuators
        
        if self.debug:
            print(f"执行器信息刷新完成，执行器数量变化: {len(actuators_before)} -> {len(actuators_after)}")
    
    def update_actuator_mode(self, actuator_id):
        """
        更新指定执行器的模式信息
        
        Args:
            actuator_id (int): 执行器ID
            
        Returns:
            bool: 如果成功更新返回True，否则返回False
        """
        if actuator_id not in self.actuators:
            if self.debug:
                print(f"无法更新模式：执行器 0x{actuator_id:02X} 不存在")
            return False
            
        try:
            mode_packet = self._build_packet(actuator_id, 0x55, b'')
            response = self._send_packet_and_receive(mode_packet)
            parsed_response = self._parse_response(response)
            
            if parsed_response['command'] == 0x55 and parsed_response['address'] == actuator_id:
                if parsed_response['data_length'] >= 1 and len(parsed_response['data']) >= 1:
                    mode = parsed_response['data'][0]
                    self.actuators[actuator_id]['mode'] = mode
                    self.actuators[actuator_id]['last_update'] = time.time()  # 更新最后更新时间
                    if self.debug:
                        mode_str = self._get_mode_string(mode)
                        print(f"执行器 0x{actuator_id:02X} 模式更新成功: {mode_str}")
                    return True
                else:
                    # 数据长度不足，保留旧模式信息，仅标记为过期
                    if self.debug:
                        print(f"执行器 0x{actuator_id:02X} 模式读取失败：无数据")
                    self.actuators[actuator_id]['last_update'] = time.time() - 10  # 标记为过期
                    return False
            else:
                if self.debug:
                    print(f"执行器 0x{actuator_id:02X} 模式查询：收到非预期响应，指令={parsed_response['command']:02X}，地址={parsed_response['address']:02X}")
                return False
                
        except Exception as e:
            if self.debug:
                print(f"执行器 0x{actuator_id:02X} 模式更新异常: {e}")
            return False
    
    def _enable_all_actuators(self):
        """
        使能所有发现的执行器
        """
        if self.debug:
            print("开始使能所有执行器...")
        
        for actuator_id in list(self.actuators.keys()):
            try:
                # 使能执行器
                if self.enable_actuator(actuator_id):
                    if self.debug:
                        print(f"执行器 0x{actuator_id:02X} 使能成功")
                else:
                    if self.debug:
                        print(f"执行器 0x{actuator_id:02X} 使能失败")
            except Exception as e:
                if self.debug:
                    print(f"执行器 0x{actuator_id:02X} 使能过程中出现异常: {e}")
    
    def enable_actuator(self, actuator_id):
        """
        使能指定ID的执行器
        指令格式: EE 06 2A 00 01 01 CRC CRC ED
        其中06为执行器ID,2A为使能指令,00 01是数据长度,01即为使能
        
        Args:
            actuator_id (int): 执行器ID
        """
        if self.debug:
            print(f"使能执行器 0x{actuator_id:02X}...")
        
        # 指令 0x2A: 使能/失能执行器
        # 数据: 0x01 (使能)
        enable_packet = self._build_packet(actuator_id, 0x2A, b'\x01')
        
        # 发送命令并等待响应，最多尝试5次
        for attempt in range(5):
            try:
                response = self._send_packet_and_receive(enable_packet)
                parsed_response = self._parse_response(response)
                
                # 检查响应是否是针对使能命令的正确响应
                if parsed_response['command'] == 0x2A and parsed_response['address'] == actuator_id:
                    # 使能命令的响应可能没有数据，只要地址和命令正确就认为成功
                    if self.debug:
                        print(f"执行器 0x{actuator_id:02X} 使能成功")
                    return True
                else:
                    # 如果收到的不是针对该执行器的使能响应，可能是其他执行器的响应，忽略并继续等待
                    if self.debug:
                        print(f"执行器 0x{actuator_id:02X} 使能尝试 {attempt+1}：收到非预期响应，指令={parsed_response['command']:02X}，地址={parsed_response['address']:02X}")
                    # 等待一小段时间再重试
                    time.sleep(0.1)
            
            except Exception as e:
                if self.debug:
                    print(f"执行器 0x{actuator_id:02X} 使能尝试 {attempt+1} 异常: {e}")
                # 等待一小段时间再重试
                time.sleep(0.1)
        
        if self.debug:
            print(f"执行器 0x{actuator_id:02X} 使能失败：超过最大尝试次数")
        return False
    
    def disable_actuator(self, actuator_id):
        """
        失能指定ID的执行器
        指令格式: EE 06 2A 00 01 00 CRC CRC ED
        其中06为执行器ID,2A为失能指令,00 01是数据长度,00即为失能
        
        Args:
            actuator_id (int): 执行器ID
        """
        if self.debug:
            print(f"失能执行器 0x{actuator_id:02X}...")
        
        # 指令 0x2A: 使能/失能执行器
        # 数据: 0x00 (失能)
        disable_packet = self._build_packet(actuator_id, 0x2A, b'\x00')
        
        # 发送命令并等待响应，最多尝试5次
        for attempt in range(5):
            try:
                response = self._send_packet_and_receive(disable_packet)
                parsed_response = self._parse_response(response)
                
                # 检查响应是否是针对失能命令的正确响应
                if parsed_response['command'] == 0x2A and parsed_response['address'] == actuator_id:
                    # 失能命令的响应可能没有数据，只要地址和命令正确就认为成功
                    if self.debug:
                        print(f"执行器 0x{actuator_id:02X} 失能成功")
                    return True
                else:
                    # 如果收到的不是针对该执行器的失能响应，可能是其他执行器的响应，忽略并继续等待
                    if self.debug:
                        print(f"执行器 0x{actuator_id:02X} 失能尝试 {attempt+1}：收到非预期响应，指令={parsed_response['command']:02X}，地址={parsed_response['address']:02X}")
                    # 等待一小段时间再重试
                    time.sleep(0.1)
            
            except Exception as e:
                if self.debug:
                    print(f"执行器 0x{actuator_id:02X} 失能尝试 {attempt+1} 异常: {e}")
                # 等待一小段时间再重试
                time.sleep(0.1)
        
        if self.debug:
            print(f"执行器 0x{actuator_id:02X} 失能失败：超过最大尝试次数")
        return False
    
    def set_actuator_mode(self, actuator_id, mode):
        """
        设置指定执行器的工作模式
        
        Args:
            actuator_id (int): 执行器ID
            mode (int): 工作模式
                0x01: 电流模式
                0x02: 速度模式
                0x03: 位置模式
                0x06: 梯形位置模式
                0x07: 梯形速度模式
                0x08: homing模式
            
        Returns:
            bool: 如果成功设置返回True，否则返回False
        """
        if self.debug:
            mode_str = self._get_mode_string(mode)
            print(f"设置执行器 0x{actuator_id:02X} 的工作模式为: {mode_str}")
        
        # 指令 0x07: 设置指定ID执行器的模式
        # 数据: 模式值
        mode_packet = self._build_packet(actuator_id, 0x07, bytes([mode]))
        
        # 发送命令并等待响应，最多尝试5次
        for attempt in range(5):
            try:
                response = self._send_packet_and_receive(mode_packet)
                parsed_response = self._parse_response(response)
                
                # 检查响应是否是针对设置模式命令的正确响应
                if parsed_response['command'] == 0x07 and parsed_response['address'] == actuator_id:
                    # 检查响应数据，0x01表示成功
                    if (parsed_response['data_length'] >= 1 and 
                        len(parsed_response['data']) >= 1 and 
                        parsed_response['data'][0] == 0x01):
                        if self.debug:
                            print(f"执行器 0x{actuator_id:02X} 模式设置成功")
                        # 更新本地存储的模式信息
                        if actuator_id in self.actuators:
                            self.actuators[actuator_id]['mode'] = mode
                            self.actuators[actuator_id]['last_update'] = time.time()
                        return True
                    else:
                        if self.debug:
                            print(f"执行器 0x{actuator_id:02X} 模式设置失败")
                        return False
                else:
                    # 如果收到的不是针对该执行器的模式设置响应，可能是其他执行器的响应，忽略并继续等待
                    if self.debug:
                        print(f"执行器 0x{actuator_id:02X} 模式设置尝试 {attempt+1}：收到非预期响应，指令={parsed_response['command']:02X}，地址={parsed_response['address']:02X}")
                    # 等待一小段时间再重试
                    time.sleep(0.1)
            
            except Exception as e:
                if self.debug:
                    print(f"执行器 0x{actuator_id:02X} 模式设置尝试 {attempt+1} 异常: {e}")
                # 等待一小段时间再重试
                time.sleep(0.1)
        
        if self.debug:
            print(f"执行器 0x{actuator_id:02X} 模式设置失败：超过最大尝试次数")
        return False
    
    def set_actuator_to_trap_position_mode(self, actuator_id):
        """
        设置指定执行器为梯形位置模式
        
        Args:
            actuator_id (int): 执行器ID
            
        Returns:
            bool: 如果成功设置返回True，否则返回False
        """
        if self.debug:
            print(f"设置执行器 0x{actuator_id:02X} 为梯形位置模式")
        
        # 指令 0x07: 设置指定ID执行器的模式
        # 数据: 0x06 (梯形位置模式)
        mode_packet = self._build_packet(actuator_id, 0x07, bytes([0x06]))
        
        # 发送命令并等待响应，最多尝试5次
        for attempt in range(5):
            try:
                response = self._send_packet_and_receive(mode_packet)
                parsed_response = self._parse_response(response)
                
                # 检查响应是否是针对设置模式命令的正确响应
                if parsed_response['command'] == 0x07 and parsed_response['address'] == actuator_id:
                    # 检查响应数据，0x01表示成功
                    if (parsed_response['data_length'] >= 1 and 
                        len(parsed_response['data']) >= 1 and 
                        parsed_response['data'][0] == 0x01):
                        if self.debug:
                            print(f"执行器 0x{actuator_id:02X} 已成功设置为梯形位置模式")
                        # 更新本地存储的模式信息
                        if actuator_id in self.actuators:
                            self.actuators[actuator_id]['mode'] = 0x06
                            self.actuators[actuator_id]['last_update'] = time.time()
                        return True
                    else:
                        if self.debug:
                            print(f"执行器 0x{actuator_id:02X} 设置梯形位置模式失败")
                        return False
                else:
                    # 如果收到的不是针对该执行器的模式设置响应，可能是其他执行器的响应，忽略并继续等待
                    if self.debug:
                        print(f"执行器 0x{actuator_id:02X} 设置梯形位置模式尝试 {attempt+1}：收到非预期响应，指令={parsed_response['command']:02X}，地址={parsed_response['address']:02X}")
                    # 等待一小段时间再重试
                    time.sleep(0.1)
            
            except Exception as e:
                if self.debug:
                    print(f"执行器 0x{actuator_id:02X} 设置梯形位置模式尝试 {attempt+1} 异常: {e}")
                # 等待一小段时间再重试
                time.sleep(0.1)
        
        if self.debug:
            print(f"执行器 0x{actuator_id:02X} 设置梯形位置模式失败：超过最大尝试次数")
        return False
    
    def set_actuator_position(self, actuator_id, position):
        """
        设置指定执行器的位置值（需要先将执行器设置为梯形位置模式）
        
        Args:
            actuator_id (int): 执行器ID
            position (float): 目标位置值（单位：转数）
            
        Returns:
            bool: 如果成功设置返回True，否则返回False
        """
        if self.debug:
            print(f"设置执行器 0x{actuator_id:02X} 的位置为 {position}R")
        
        # 将位置值转换为IQ24格式
        # 公式：目标位置 * (2^24)
        iq24_position = int(position * (2**24)) & 0xFFFFFFFF  # 确保是32位值
        position_bytes = struct.pack('>I', iq24_position)  # 大端序
        
        if self.debug:
            print(f"位置值 {position}R 转换为IQ24格式: 0x{iq24_position:08X}")
        
        # 指令 0x0A: 设置当前位置值
        # 数据: 位置值（IQ24格式，4字节）
        position_packet = self._build_packet(actuator_id, 0x0A, position_bytes)
        
        # 发送命令并等待响应，最多尝试5次
        for attempt in range(5):
            try:
                response = self._send_packet_and_receive(position_packet)
                parsed_response = self._parse_response(response)
                
                # 检查响应是否是针对设置位置命令的正确响应
                # 注意：根据文档，指令0x0A可能没有返回数据
                if parsed_response['command'] == 0x0A and parsed_response['address'] == actuator_id:
                    if self.debug:
                        print(f"执行器 0x{actuator_id:02X} 位置设置成功")
                    return True
                else:
                    # 如果收到的不是针对该执行器的位置设置响应，可能是其他执行器的响应，忽略并继续等待
                    if self.debug:
                        print(f"执行器 0x{actuator_id:02X} 位置设置尝试 {attempt+1}：收到非预期响应，指令={parsed_response['command']:02X}，地址={parsed_response['address']:02X}")
                    # 等待一小段时间再重试
                    time.sleep(0.1)
            
            except Exception as e:
                if self.debug:
                    print(f"执行器 0x{actuator_id:02X} 位置设置尝试 {attempt+1} 异常: {e}")
                # 等待一小段时间再重试
                time.sleep(0.1)
        
        if self.debug:
            print(f"执行器 0x{actuator_id:02X} 位置设置失败：超过最大尝试次数")
        return False
    
    def get_actuator_position(self, actuator_id, use_crc=False):
        """
        获取指定执行器的当前位置值
        
        Args:
            actuator_id (int): 执行器ID
            use_crc (bool): 是否使用CRC校验码，默认为False
            
        Returns:
            float: 当前位置值（单位：转数），如果失败则返回None
        """
        if self.debug:
            print(f"获取执行器 0x{actuator_id:02X} 的当前位置...")
        
        # 指令 0x06: 读取当前位置值
        position_packet = self._build_packet(actuator_id, 0x06, b'', no_crc=not use_crc)
        
        # 发送命令并等待响应，最多尝试5次
        for attempt in range(5):
            try:
                response = self._send_packet_and_receive(position_packet)
                parsed_response = self._parse_response(response)
                
                # 检查响应是否是针对读取位置命令的正确响应
                if parsed_response['command'] == 0x06 and parsed_response['address'] == actuator_id:
                    # 检查是否有足够的数据（4字节位置数据）
                    if parsed_response['data_length'] >= 4 and len(parsed_response['data']) >= 4:
                        # 位置数据是IQ24格式，大端序
                        position_bytes = parsed_response['data'][:4]
                        iq24_position = struct.unpack('>I', position_bytes)[0]
                        
                        # 将IQ24格式转换为实际位置值（转数）
                        # 公式：实际位置 = IQ24值 / (2^24)
                        position = iq24_position / (2**24)
                        
                        # 处理负数情况（如果最高位为1，则为负数）
                        if iq24_position >= 2**31:
                            position = position - 2**8  # 调整为负数范围
                        
                        if self.debug:
                            print(f"执行器 0x{actuator_id:02X} 当前位置: {position}R (IQ24: 0x{iq24_position:08X})")
                        return position
                    else:
                        if self.debug:
                            print(f"执行器 0x{actuator_id:02X} 位置读取失败：数据不足")
                        return None
                else:
                    # 如果收到的不是针对该执行器的位置响应，可能是其他执行器的响应，忽略并继续等待
                    if self.debug:
                        print(f"执行器 0x{actuator_id:02X} 位置读取尝试 {attempt+1}：收到非预期响应，指令={parsed_response['command']:02X}，地址={parsed_response['address']:02X}")
                    # 等待一小段时间再重试
                    time.sleep(0.1)
            
            except Exception as e:
                if self.debug:
                    print(f"执行器 0x{actuator_id:02X} 位置读取尝试 {attempt+1} 异常: {e}")
                # 等待一小段时间再重试
                time.sleep(0.1)
        
        if self.debug:
            print(f"执行器 0x{actuator_id:02X} 位置读取失败：超过最大尝试次数")
        return None
    
    def get_all_actuators_positions(self):
        """
        获取所有执行器的当前位置值
        
        Returns:
            dict: 包含所有执行器位置信息的字典，格式为 {actuator_id: position}
        """
        positions = {}
        if self.debug:
            print("开始获取所有执行器的当前位置...")
        
        for actuator_id in list(self.actuators.keys()):
            try:
                position = self.get_actuator_position(actuator_id)
                positions[actuator_id] = position
                if position is not None:
                    if self.debug:
                        print(f"执行器 0x{actuator_id:02X} 当前位置: {position}R")
                else:
                    if self.debug:
                        print(f"执行器 0x{actuator_id:02X} 位置读取失败")
            except Exception as e:
                if self.debug:
                    print(f"执行器 0x{actuator_id:02X} 位置读取异常: {e}")
                positions[actuator_id] = None
        
        return positions
    
    def refresh_actuator_mode(self, actuator_id):
        """
        刷新指定执行器的模式信息
        
        Args:
            actuator_id (int): 执行器ID
            
        Returns:
            int: 返回执行器的当前模式，如果失败则返回None
        """
        if self.debug:
            print(f"刷新执行器 0x{actuator_id:02X} 的模式信息...")
        
        # 获取执行器的当前模式
        mode = self._get_actuator_mode(actuator_id)
        
        if mode is not None:
            if actuator_id in self.actuators:
                self.actuators[actuator_id]['mode'] = mode
                self.actuators[actuator_id]['last_update'] = time.time()
                if self.debug:
                    mode_str = self._get_mode_string(mode)
                    print(f"执行器 0x{actuator_id:02X} 模式刷新成功: {mode_str}")
            return mode
        
        if self.debug:
            print(f"执行器 0x{actuator_id:02X} 模式刷新失败")
        return None
    
    def close(self):
        """
        关闭socket连接
        """
        if not self._cleaned_up:
            self.disable_all_actuators()
            self.socket.close()
            self._cleaned_up = True
            if self.debug:
                print("Socket连接已关闭")
    
    def _set_all_actuators_to_position_mode(self):
        """
        将所有执行器的工作模式设置为位置模式 (0x03)
        """
        if self.debug:
            print("开始将所有执行器设置为位置模式...")
        
        for actuator_id in list(self.actuators.keys()):
            try:
                # 将执行器设置为位置模式
                if self.set_actuator_mode(actuator_id, 0x03):
                    if self.debug:
                        print(f"执行器 0x{actuator_id:02X} 已设置为位置模式")
                else:
                    if self.debug:
                        print(f"执行器 0x{actuator_id:02X} 设置为位置模式失败")
            except Exception as e:
                if self.debug:
                    print(f"执行器 0x{actuator_id:02X} 设置为位置模式时出现异常: {e}")
    
    def __del__(self):
        """
        析构函数，在对象销毁时失能所有执行器
        """
        try:
            if not self._cleaned_up:
                self.disable_all_actuators()
                if self.debug:
                    print("sdk已退出。")
        except:
            pass  # 忽略析构时的任何异常
    
    def disable_all_actuators(self):
        """
        失能所有执行器
        """
        if self.debug:
            print("开始失能所有执行器...")
        
        for actuator_id in list(self.actuators.keys()):
            try:
                # 失能执行器
                if self.disable_actuator(actuator_id):
                    if self.debug:
                        print(f"执行器 0x{actuator_id:02X} 失能成功")
                else:
                    if self.debug:
                        print(f"执行器 0x{actuator_id:02X} 失能失败")
            except Exception as e:
                if self.debug:
                    print(f"执行器 0x{actuator_id:02X} 失能过程中出现异常: {e}")

if __name__ == "__main__":
    # 示例用法
    try:
        actuator = MintascaActuator(debug=True)
    except Exception as e:
        print(f"初始化执行器通信失败: {e}")