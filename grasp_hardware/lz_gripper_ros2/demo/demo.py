#!/usr/bin/env python3
"""
灵掌机械手直接串口控制程序
直接通过串口与控制板通讯，不依赖ROS2

========== 切换左右手控制说明 ==========
1. 机械手ID配置：
   - 左手：Left_Hand = 2  
   - 右手：Right_Hand = 1

2. 如要换成右手控制：
   - 将main()函数中所有的 Left_Hand 改为 Right_Hand
   - 共计15处需要修改，都已标记 🔄 注释

3. 端口配置（如需要）：
   - 当前使用：/dev/ttyUSB2
   - 如右手使用不同端口，需修改 DirectGripperController() 的 port 参数

=========================================
"""
import serial
import time
import struct
import sys

# 控制命令类型常量
Finger_Positions = 1    # 手指位置控制
gripper_Command = 1     # 机械手命令
Finger_Velocity = 3     # 手指速度控制
Finger_Effort = 4       # 手指力控制
Read_Finger_Pressure = 5  # 读取手指压力值

# 机械手ID - 这是区分左右手的关键参数
Left_Hand = 2          # 左手ID (Modbus从机地址2)
Right_Hand = 1         # 右手ID (Modbus从机地址1)

# 控制模式
Emergency = 1          # 紧急模式
Normal = 0            # 正常模式

class DirectGripperController:
    """直接串口控制的机械手控制器"""
    
    def __init__(self, port='/dev/ttyUSB2', baudrate=115200):
        """
        初始化串口连接
        
        参数:
            port: 串口设备路径
                 - 当前默认: /dev/ttyUSB2
                 - 如右手使用不同端口，在此修改，例如: '/dev/ttyUSB0'
            baudrate: 波特率，通常为115200
        """
        self.port = port
        self.baudrate = baudrate
        self.serial_conn = None
        self.running = True
        self.connect()
    
    def connect(self):
        """连接到串口"""
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1.0,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False
            )
            print(f"成功连接到串口: {self.port}, 波特率: {self.baudrate}")
            return True
        except Exception as e:
            print(f"无法连接到串口 {self.port}: {str(e)}")
            return False
    
    def disconnect(self):
        """断开串口连接"""
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            print("串口连接已断开")
    
    def modbus_crc(self, data):
        """计算Modbus CRC校验码"""
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        return crc
    
    def send_modbus_command(self, slave_addr, func_code, start_addr, data_values):
        """发送Modbus写多个寄存器命令"""
        if not self.serial_conn or not self.serial_conn.is_open:
            print("串口未连接")
            return False
        
        try:
            # 清空输入输出缓冲区
            self.serial_conn.reset_input_buffer()
            self.serial_conn.reset_output_buffer()
            
            # 构建Modbus RTU数据包
            num_registers = len(data_values)
            byte_count = num_registers * 2
            
            # 命令头部
            command = bytearray([
                slave_addr,        # 从机地址
                func_code,         # 功能码 0x10 (写多个寄存器)
                (start_addr >> 8) & 0xFF,  # 起始地址高字节
                start_addr & 0xFF,         # 起始地址低字节
                (num_registers >> 8) & 0xFF,  # 寄存器数量高字节
                num_registers & 0xFF,         # 寄存器数量低字节
                byte_count                    # 字节数
            ])
            
            # 添加数据值
            for value in data_values:
                command.extend([
                    (value >> 8) & 0xFF,  # 高字节
                    value & 0xFF          # 低字节
                ])
            
            # 计算并添加CRC
            crc = self.modbus_crc(command)
            command.extend([
                crc & 0xFF,        # CRC低字节
                (crc >> 8) & 0xFF  # CRC高字节
            ])
            
            # 发送命令
            bytes_written = self.serial_conn.write(command)
            self.serial_conn.flush()
            
            # 等待响应
            time.sleep(0.1)
            
            # 检查有多少数据可读
            waiting = self.serial_conn.in_waiting
            
            if waiting > 0:
                # 读取响应（期望8字节响应）
                response = self.serial_conn.read(waiting)
                
                if len(response) >= 6:  # 最小有效响应长度
                    # 验证响应
                    if response[0] == slave_addr and response[1] == func_code:
                        return True
                    else:
                        print(f"响应验证失败: 从机={response[0]}, 功能码={response[1]}")
                else:
                    print(f"响应长度不足: {len(response)} 字节")
            else:
                print("没有收到响应")
            
            return False
            
        except Exception as e:
            print(f"发送命令失败: {str(e)}")
            return False
    
    def read_pressure_values(self, slave_addr):
        """读取压力值"""
        if not self.serial_conn or not self.serial_conn.is_open:
            print("串口未连接")
            return None
        
        try:
            # 清空输入缓冲区
            self.serial_conn.reset_input_buffer()
            
            # 构建读保持寄存器命令 (功能码 0x03)
            start_addr = 18  # 压力值寄存器起始地址
            num_registers = 5  # 5个手指的压力值
            
            command = bytearray([
                slave_addr,        # 从机地址
                0x03,             # 功能码 (读保持寄存器)
                (start_addr >> 8) & 0xFF,  # 起始地址高字节
                start_addr & 0xFF,         # 起始地址低字节
                (num_registers >> 8) & 0xFF,  # 寄存器数量高字节
                num_registers & 0xFF,         # 寄存器数量低字节
            ])
            
            # 计算并添加CRC
            crc = self.modbus_crc(command)
            command.extend([
                crc & 0xFF,        # CRC低字节
                (crc >> 8) & 0xFF  # CRC高字节
            ])
            
            # 发送命令
            self.serial_conn.write(command)
            self.serial_conn.flush()
            
            # 等待响应
            time.sleep(0.1)
            
            # 读取响应，使用更简单的方式
            response = self.serial_conn.read(13)  # 期望13字节响应
            
            if len(response) >= 13:
                # 验证响应头
                if response[0] == slave_addr and response[1] == 0x03:
                    byte_count = response[2]
                    if byte_count == 10:  # 5个寄存器 * 2字节/寄存器
                        # 直接解析数据，不验证CRC（由于CRC可能有问题）
                        pressures = []
                        for i in range(5):
                            high_byte = response[3 + i*2]
                            low_byte = response[4 + i*2]
                            pressure = (high_byte << 8) | low_byte
                            pressures.append(pressure)
                        return pressures
                    else:
                        print(f"字节数错误: 期望10, 收到{byte_count}")
                else:
                    print(f"响应头错误: 从机={response[0]}, 功能码={response[1]}")
            else:
                print(f"响应长度不足: {len(response)} 字节")
            
            return None
            
        except Exception as e:
            print(f"读取压力值失败: {str(e)}")
            return None
    
    def set_register(self, data):
        """设置寄存器值（兼容原有接口）"""
        if not self.running:
            return False
        
        # 适配两种数据格式：
        # 格式1：[命令类型, 机械手ID, 延迟, 6个手指值] = 9个元素
        # 格式2：[命令类型, 机械手ID, 模式, 延迟, 6个手指值] = 10个元素
        
        if len(data) == 9:
            # 9个参数格式：[命令类型, 机械手ID, 延迟, 6个手指值]
            cmd_type = int(data[0])
            hand_id = int(data[1])
            delay = int(data[2])
            finger_values = [int(data[i]) for i in range(3, 9)]
        elif len(data) == 10:
            # 10个参数格式：[命令类型, 机械手ID, 模式, 延迟, 6个手指值]
            cmd_type = int(data[0])
            hand_id = int(data[1])
            mode = int(data[2])
            delay = int(data[3])
            finger_values = [int(data[i]) for i in range(4, 10)]
        else:
            print(f"数据长度错误: {len(data)}，期望9或10个参数")
            return False
        
        print(f"执行命令: 类型={cmd_type}, 手ID={hand_id}, 延迟={delay}")
        
        # 根据命令类型设置起始地址
        if cmd_type == Finger_Positions:
            start_addr = 0  # 位置寄存器起始地址
        elif cmd_type == Finger_Velocity:
            start_addr = 6  # 速度寄存器起始地址  
        elif cmd_type == Finger_Effort:
            start_addr = 12  # 力矩寄存器起始地址
        else:
            print(f"未知命令类型: {cmd_type}")
            return False
        
        # 发送Modbus命令
        success = self.send_modbus_command(hand_id, 0x10, start_addr, finger_values)
        
        if delay > 0:
            time.sleep(delay / 1000.0)  # 延迟单位转换为秒
        
        return success
    
    def get_fingerpressure(self, hand_id):
        """获取手指压力值（兼容原有接口）"""
        if not self.running:
            return False
        
        pressures = self.read_pressure_values(hand_id)
        if pressures:
            print(f'压力值 拇指: {pressures[0]} 食指: {pressures[1]} 中指: {pressures[2]} 无名指: {pressures[3]} 小指: {pressures[4]}')
            return True
        else:
            print("读取压力值失败")
            return False
    
    def send_request(self, a, b, c, d, e):
        """发送请求（兼容原有接口）"""
        # 将参数转换为set_register格式
        data = [a, b, c, d] + e
        return self.set_register(data)
    
    def service_test(self):
        """测试服务功能（兼容原有接口）"""
        success = self.send_request(Finger_Positions, Left_Hand, 0, 544, [500, 500, 500, 300, 300, 300])  # 🔄 改为 Right_Hand
        if success:
            print("服务测试成功")
        else:
            print("服务测试失败")

def main():
    print("启动灵掌机械手直接控制程序...")
    
    # 创建控制器
    controller = DirectGripperController()
    
    if not controller.serial_conn or not controller.serial_conn.is_open:
        print("无法连接到机械手，请检查:")
        print("1. 串口设备是否存在: /dev/ttyUSB2")
        print("2. 设备权限是否正确")
        print("3. 机械手是否正确连接并上电")
        return
    
    try:
        print("开始执行动作序列...")
        
        # 数组各部分含义：灵巧手id（左手2,右手1），控制动作持续时间（单位1ms）,大拇指反转角度（最小0,最大1000），
        # 大拇指弯曲幅度，食指弯曲幅度，中指弯曲幅度，无名指弯曲幅度，小拇指弯曲幅度
        
        # ======== 如要换成右手控制，请将以下所有 Left_Hand 改为 Right_Hand ========
        # Left_Hand = 2 (左手)
        # Right_Hand = 1 (右手)
        
        print("设置初始力矩...")
        controller.set_register([Finger_Effort, Left_Hand, Normal, 1, 1000, 1000, 1000, 1000, 1000, 1000])  # 🔄 改为 Right_Hand
        time.sleep(0.1)
        
        print("设置初始速度...")
        controller.set_register([Finger_Velocity, Left_Hand, Normal, 1, 1000, 1000, 1000, 1000, 1000, 1000])  # 🔄 改为 Right_Hand
        time.sleep(0.1)
        
        print("设置位置1...")
        controller.set_register([Finger_Positions, Left_Hand, 544, 500, 500, 500, 300, 300, 300])  # 🔄 改为 Right_Hand
        time.sleep(1.0)
        
        print("设置位置2...")
        controller.set_register([Finger_Positions, Left_Hand, 530, 100, 100, 100, 300, 300, 300])  # 🔄 改为 Right_Hand
        time.sleep(1.0)
        
        print("调整速度...")
        controller.set_register([Finger_Velocity, Left_Hand, Normal, 1, 400, 400, 400, 400, 400, 400])  # 🔄 改为 Right_Hand
        time.sleep(0.1)
        
        print("设置位置3...")
        controller.set_register([Finger_Positions, Left_Hand, 1030, 500, 500, 500, 500, 500, 500])  # 🔄 改为 Right_Hand
        time.sleep(1.0)
        
        print("设置位置4...")
        controller.set_register([Finger_Positions, Left_Hand, 630, 100, 100, 100, 100, 100, 100])  # 🔄 改为 Right_Hand
        time.sleep(1.0)
        
        print("调整速度和力矩...")
        controller.set_register([Finger_Velocity, Left_Hand, Normal, 1, 600, 600, 600, 600, 600, 600])  # 🔄 改为 Right_Hand
        time.sleep(0.1)
        controller.set_register([Finger_Effort, Left_Hand, Normal, 1, 1000, 1000, 1000, 1000, 1000, 1000])  # 🔄 改为 Right_Hand
        time.sleep(0.1)
        
        print("执行抓取动作...")
        controller.set_register([Finger_Positions, Left_Hand, 1030, 400, 400, 400, 400, 400, 400])  # 🔄 改为 Right_Hand
        time.sleep(1.0)
        controller.set_register([Finger_Positions, Left_Hand, 1030, 0, 0, 0, 0, 0, 0])  # 🔄 改为 Right_Hand
        time.sleep(1.0)
        
        print("调整力矩...")
        controller.set_register([Finger_Effort, Left_Hand, Normal, 1, 500, 500, 500, 500, 500, 500])  # 🔄 改为 Right_Hand
        time.sleep(0.1)
        controller.set_register([Finger_Positions, Left_Hand, 1030, 400, 400, 400, 400, 400, 400])  # 🔄 改为 Right_Hand
        time.sleep(1.0)
        
        print("开始监控压力值 (按Ctrl+C退出)...")
        while controller.running:
            try:
                controller.get_fingerpressure(Left_Hand)  # 🔄 改为 Right_Hand
                time.sleep(0.5)  # 每0.5秒读取一次
            except KeyboardInterrupt:
                print("\n收到中断信号，正在停止...")
                controller.running = False
                break
                 
    except Exception as e:
        print(f"程序执行出错: {str(e)}")
    except KeyboardInterrupt:
        print("\n收到中断信号，正在停止...")
    finally:
        print("关闭连接...")
        controller.disconnect()

if __name__ == '__main__':
    main() 