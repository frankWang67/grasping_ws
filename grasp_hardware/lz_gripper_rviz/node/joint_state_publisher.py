#!/usr/bin/env python3
"""
🤖 机械手关节状态发布节点

功能概述：
- 🎯 接收位置控制命令并实现平滑运动插值
- 📊 发布关节状态到RViz进行实时可视化  
- 🤏 智能压力传感器监控和安全停止
- ⚙️  支持6自由度机械手控制（拇指2轴+4指各1轴）
- 🔄 支持左右手选择控制

核心特性：
• 余弦插值平滑运动
• 压力感知安全停止（仅在弯曲时生效）
• 运动方向智能识别
• 实时压力数据缓存和监控
• 左右手切换支持
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32MultiArray
from sensor_msgs.msg import JointState
from lz_gripper_ros2.srv import ServerMsg
from math import pi, cos
import time
from rclpy.time import Time
from typing import Dict, List, Tuple, Optional

class GripperJointStatePublisher(Node):
    """机械手关节状态发布节点类
    
    负责处理机械手的运动控制和状态发布：
    1. 接收位置控制命令并进行处理
    2. 实现平滑的关节运动（余弦插值）
    3. 发布关节状态到RViz
    4. 处理压力传感器数据
    5. 支持左右手选择控制
    
    发布话题:
    - joint_states (sensor_msgs/JointState): 关节状态
    
    订阅话题:
    - write_register (std_msgs/Float32MultiArray): 位置控制命令
    - lz_gripper_states (sensor_msgs/JointState): 压力传感器数据
    """
    
    def __init__(self):
        """初始化节点、配置和通信组件"""
        super().__init__('gripper_joint_state_publisher')
        
        # 声明参数
        self.declare_parameter('hand_side', 'left')
        
        # 读取手边参数并设置手ID
        hand_side = self.get_parameter('hand_side').get_parameter_value().string_value
        if hand_side.lower() == 'left':
            self.selected_hand_id = 2  # 左手ID
            self.hand_name = "左手"
        elif hand_side.lower() == 'right':
            self.selected_hand_id = 1  # 右手ID
            self.hand_name = "右手"
        else:
            self.get_logger().warn(f'未知的手边参数: {hand_side}，默认使用左手')
            self.selected_hand_id = 2  # 左手ID
            self.hand_name = "左手"
            
        self.get_logger().info(f'关节状态发布器配置: {self.hand_name} (ID: {self.selected_hand_id})')
        
        # 定义关节名称列表（按控制顺序排列）
        self.joint_names = [
            'thumb_joint1',   # 拇指翻转关节
            'thumb_joint2',   # 拇指第二关节
            'thumb_joint3',   # 拇指第三关节
            'index_joint1',   # 食指第一关节
            'index_joint2',   # 食指第二关节
            'middle_joint1',  # 中指第一关节
            'middle_joint2',  # 中指第二关节
            'ring_joint1',    # 无名指第一关节
            'ring_joint2',    # 无名指第二关节
            'little_joint1',  # 小指第一关节
            'little_joint2'   # 小指第二关节
        ]
        
        # 定义手指关节映射（用于运动控制）
        self.finger_joints = {
            2: ['index_joint1', 'index_joint2'],    # 食指关节组
            3: ['middle_joint1', 'middle_joint2'],  # 中指关节组
            4: ['ring_joint1', 'ring_joint2'],      # 无名指关节组
            5: ['little_joint1', 'little_joint2']   # 小指关节组
        }
        
        # 初始化关节状态存储
        self.current_angles = {name: 0.0 for name in self.joint_names}
        self.target_angles = {name: 0.0 for name in self.joint_names}
        
        # 电机状态存储
        self.current_motor_positions = [0.0] * 6     # 当前电机位置
        self.target_motor_positions = [0.0] * 6      # 目标电机位置
        self.start_motor_positions = [0.0] * 6       # 运动开始位置
        self.movement_start_times = [None] * 6       # 每个电机的运动开始时间
        self.movement_durations = [0.0] * 6          # 每个电机的运动持续时间
        self.motors_moving = [False] * 6             # 每个电机的运动状态
        
        # 压力传感器相关状态
        self.finger_contact = {
            'thumb': False,
            'index': False, 
            'middle': False,
            'ring': False,
            'little': False
        }
        
        # 新增：压力停止状态跟踪
        self.pressure_stopped = [False] * 6  # 每个电机是否因压力而停止
        self.pressure_threshold = 850        # 压力阈值（调整到850）
        
        # 添加压力数据缓存
        self.last_pressure_values = [750.0] * 5  # 缓存最新的5个手指压力值
        self.last_pressure_time = self.get_clock().now()
        
        # 添加运动方向跟踪（用于智能压力停止）
        self.movement_directions = [0] * 6  # 1=弯曲, -1=展开, 0=无运动
        
        # 添加抓取检测状态
        self.object_grasped = False          # 是否抓取到物体
        self.grasped_fingers = []            # 抓取到物体的手指列表
        self.action_should_complete = False  # 是否应该完成动作
        
        # 更新手ID支持
        self.current_hand_id = self.selected_hand_id  # 使用选定的手ID
        
        # 手指到电机的映射（用于压力停止控制）
        self.finger_to_motor_map = {
            'thumb': [0, 1],     # 拇指对应电机0和1
            'index': [2],        # 食指对应电机2
            'middle': [3],       # 中指对应电机3
            'ring': [4],         # 无名指对应电机4
            'little': [5]        # 小指对应电机5
        }
        
        # 运动控制参数
        self.last_update_time = self.get_clock().now()
        self.thumb_movement_duration = 4.0   # 拇指满行程时间（秒）
        self.finger_movement_duration = 8.0  # 其他手指满行程时间（秒）
        self.is_initialized = False          # 初始化状态标志
        self._was_moving = False             # 运动状态跟踪标志
        
        # 声明ROS参数
        self.declare_parameter('pressure_threshold', 850)  # 压力阈值参数（调整到850）
        self.declare_parameter('enable_pressure_stopping', True)  # 是否启用压力停止功能
        
        # 获取参数值
        self.pressure_threshold = self.get_parameter('pressure_threshold').value
        self.enable_pressure_stopping = self.get_parameter('enable_pressure_stopping').value
        
        self.get_logger().info(f'压力停止功能: {"启用" if self.enable_pressure_stopping else "禁用"}')
        self.get_logger().info(f'压力阈值设置为: {self.pressure_threshold}')
        self.get_logger().info(f'关节状态发布节点启动完成！版本: v2.1 - {self.hand_name}')
        
        # 创建通信组件
        self._setup_communications()
        
        # 初始化机械手
        self.get_logger().info(f'正在初始化{self.hand_name}到完全展开状态...')
        self.initialize_gripper()

    def _setup_communications(self):
        """设置ROS2通信组件（发布者、订阅者和服务）"""
        # 创建关节状态发布者
        self.joint_state_pub = self.create_publisher(
            JointState,
            'joint_states',
            10
        )
        
        # 创建命令发布者
        self.command_pub = self.create_publisher(
            Float32MultiArray,
            'write_register',
            10
        )
        
        # 创建命令订阅者
        self.write_subscription = self.create_subscription(
            Float32MultiArray,
            'write_register',
            self.finger_position_callback,
            10
        )
        
        # 创建压力传感器数据订阅者
        self.pressure_subscription = self.create_subscription(
            JointState,
            'lz_gripper_states',
            self.pressure_callback,
            10
        )
        
        # 创建服务客户端
        self.cli = self.create_client(ServerMsg, 'lz_gripper_service')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待lz_gripper_service服务可用...')
        self.req = ServerMsg.Request()
        
        # 创建定时器（100Hz更新频率）
        self.timer = self.create_timer(1.0/100.0, self.update_joint_states)
        
        # 重新启用压力监控定时器（10Hz，降低频率避免干扰）
        self.pressure_timer = self.create_timer(1.0/10.0, self.request_pressure_data)

    def initialize_gripper(self) -> bool:
        """初始化机械手到完全展开状态
        
        执行步骤：
        1. 设置电机速度和力矩
        2. 发送位置控制命令
        3. 初始化内部状态
        4. 发布初始关节状态
        
        Returns:
            bool: 初始化是否成功
        """
        try:
            # 设置电机速度（较低速度以确保安全）
            speed_msg = Float32MultiArray()
            speed_msg.data = [3.0, float(self.selected_hand_id), 0.0, 1.0, 200.0, 200.0, 200.0, 200.0, 200.0, 200.0]
            self.command_pub.publish(speed_msg)
            time.sleep(0.1)

            # 设置电机力矩（适中的力矩）
            torque_msg = Float32MultiArray()
            torque_msg.data = [4.0, float(self.selected_hand_id), 0.0, 1.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0]
            self.command_pub.publish(torque_msg)
            time.sleep(0.1)

            # 发送位置控制命令（全部展开）
            pos_msg = Float32MultiArray()
            pos_msg.data = [1.0, float(self.selected_hand_id), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            self.command_pub.publish(pos_msg)

            # 初始化内部状态
            current_time = self.get_clock().now()
            self._reset_motor_states(current_time)
            self._reset_joint_angles()
            
            # 发布初始关节状态
            self._publish_joint_state(current_time)
            
            self.is_initialized = True
            self.get_logger().info(f'{self.hand_name}初始化完成')
            return True

        except Exception as e:
            self.get_logger().error(f'{self.hand_name}初始化失败: {str(e)}')
            return False

    def _reset_motor_states(self, current_time: Time):
        """重置所有电机状态
        
        Args:
            current_time: 当前时间戳
        """
        for i in range(6):
            self.current_motor_positions[i] = 0.0
            self.target_motor_positions[i] = 0.0
            self.start_motor_positions[i] = 0.0
            self.movement_start_times[i] = current_time
            self.motors_moving[i] = False

    def _reset_joint_angles(self):
        """重置所有关节角度到0"""
        for name in self.joint_names:
            self.current_angles[name] = 0.0
            self.target_angles[name] = 0.0

    def _publish_joint_state(self, current_time: Time):
        """发布关节状态消息
        
        Args:
            current_time: 当前时间戳
        """
        msg = JointState()
        msg.header.stamp = current_time.to_msg()
        msg.header.frame_id = "base_link"
        msg.name = self.joint_names
        msg.position = [self.current_angles[name] for name in self.joint_names]
        msg.velocity = [0.0] * len(self.joint_names)
        msg.effort = [0.0] * len(self.joint_names)
        self.joint_state_pub.publish(msg)

    def update_joint_states(self):
        """更新并发布关节状态（100Hz调用）
        
        主要功能：
        1. 计算每个电机的当前位置（基于余弦插值）
        2. 更新关节角度
        3. 发布最新的关节状态
        """
        if not self.is_initialized:
            return
            
        current_time = self.get_clock().now()
        
        # 更新电机位置
        self._update_motor_positions(current_time)
        
        # 更新关节角度
        self._update_joint_angles()
        
        # 发布关节状态
        self._publish_joint_state(current_time)
        self.last_update_time = current_time

    def _update_motor_positions(self, current_time: Time):
        """更新所有电机的当前位置
        
        使用余弦插值实现平滑运动，并支持基于压力传感器的停止控制
        
        Args:
            current_time: 当前时间戳
        """
        any_motor_moving = False
        movement_info = []
        
        for i in range(6):
            if self.motors_moving[i]:
                any_motor_moving = True
                
                # 检查是否因压力而停止
                if self.pressure_stopped[i]:
                    # 压力停止：立即停止运动并保持当前位置
                    self.motors_moving[i] = False
                    self.target_motor_positions[i] = self.current_motor_positions[i]
                    self.get_logger().info(f'电机 {i} 因压力超过阈值而停止，当前位置: {self.current_motor_positions[i]:.1f}')
                    continue
                
                # 在运动过程中检查压力数据（使用缓存）
                self._check_cached_pressure_for_motor(i)
                
                # 计算运动进度
                elapsed = float((current_time - self.movement_start_times[i]).nanoseconds) / 1e9
                duration = self.movement_durations[i]
                
                if duration > 0:
                    # 使用余弦插值计算平滑进度
                    progress = min(elapsed / duration, 1.0)
                    smooth_progress = (1 - cos(progress * pi)) / 2
                    
                    # 更新电机位置
                    start_pos = self.start_motor_positions[i]
                    target_pos = self.target_motor_positions[i]
                    new_position = start_pos + (target_pos - start_pos) * smooth_progress
                    
                    # 如果因压力停止，不允许进一步收缩（增加位置值）
                    if self.pressure_stopped[i] and new_position > self.current_motor_positions[i]:
                        # 保持当前位置，不继续收缩
                        pass
                    else:
                        self.current_motor_positions[i] = new_position
                    
                    # 检查运动是否完成
                    if progress >= 1.0:
                        self.motors_moving[i] = False
                        if not self.pressure_stopped[i]:
                            self.current_motor_positions[i] = target_pos
                            # 确定电机类型
                            if i == 0:
                                motor_type = "拇指翻转"
                            elif i == 1:
                                motor_type = "拇指弯曲"
                            else:
                                finger_names = ["食指", "中指", "无名指", "小指"]
                                motor_type = finger_names[i-2]
                            self.get_logger().info(f'电机 {i} ({motor_type}) 运动完成，最终位置: {self.current_motor_positions[i]:.1f}')
                else:
                    # 暂时禁用复杂的抓取检测快速完成逻辑
                    pass
        
        # 检查是否所有运动都已完成
        if not any_motor_moving and hasattr(self, '_was_moving') and self._was_moving:
            self._handle_movement_completion()
        
        # 记录运动状态
        self._was_moving = any_motor_moving

    def _handle_movement_completion(self):
        """处理正常的运动完成"""
        self.get_logger().info('======== 运动完成 ========')
        self.get_logger().info('所有电机运动完成！')
        final_positions = [f"{pos:.1f}" for pos in self.current_motor_positions]
        self.get_logger().info(f'最终位置: [{", ".join(final_positions)}]')
        self.get_logger().info('==========================')
        
        # 重置抓取状态
        self.object_grasped = False
        self.grasped_fingers = []
        self.action_should_complete = False
    
    def _handle_grasp_completion(self):
        """处理因抓取检测而完成的动作"""
        self.get_logger().info('======== 抓取检测完成 ========')
        if self.grasped_fingers:
            grasped_info = ", ".join(self.grasped_fingers)
            self.get_logger().info(f'检测到物体！抓取手指: {grasped_info}')
        self.get_logger().info('动作因抓取检测而智能终止')
        final_positions = [f"{pos:.1f}" for pos in self.current_motor_positions]
        self.get_logger().info(f'最终位置: [{", ".join(final_positions)}]')
        self.get_logger().info('===============================')
        
        # 重置抓取状态
        self.object_grasped = False
        self.grasped_fingers = []
        self.action_should_complete = False

    def request_pressure_data(self):
        """定期请求压力数据（10Hz）
        
        使用与hand_controller_node相同的方法通过服务获取压力数据
        """
        # 只在有电机运动时才进行压力检查
        if not any(self.motors_moving):
            return
            
        self.get_logger().info(f'主动请求压力数据 ({self.hand_name})')
            
        try:
            # 使用与hand_controller_node相同的方法
            self.req.messageid = 5  # Read_Finger_Pressure
            self.req.id = self.current_hand_id  # 使用当前手ID
            self.req.flag = 0
            self.req.delay = 0
            
            # 发送异步请求并绑定回调
            future = self.cli.call_async(self.req)
            future.add_done_callback(self._pressure_request_callback)
            
        except Exception as e:
            self.get_logger().warn(f'压力数据请求失败: {str(e)}')
    
    def _pressure_request_callback(self, future):
        """处理压力数据请求的回调"""
        try:
            response = future.result()
            if response is not None and hasattr(response, 'pressure'):
                pressure_values = [float(p) for p in response.pressure]
                self.get_logger().info(f'主动获取压力数据 ({self.hand_name}): {pressure_values}')
                
                # 更新缓存的压力数据
                if len(pressure_values) >= 5:
                    self.last_pressure_values = pressure_values[:5]
                    self.last_pressure_time = self.get_clock().now()
            else:
                self.get_logger().debug('压力数据响应为空')
        except Exception as e:
            self.get_logger().debug(f'压力数据请求回调失败: {str(e)}')

    def _update_joint_angles(self):
        """更新所有关节的角度
        
        基于当前电机位置计算对应的关节角度
        """
        # 更新拇指翻转关节
        self.current_angles['thumb_joint1'] = self.convert_motor_value_to_angle(
            self.current_motor_positions[0], True)
        
        # 更新拇指弯曲关节组
        thumb2_angle, thumb3_angle = self.convert_thumb_angles(self.current_motor_positions[1])
        self.current_angles['thumb_joint2'] = thumb2_angle
        self.current_angles['thumb_joint3'] = thumb3_angle
        
        # 更新其他手指关节
        for i in range(2, 6):
            base_joint, second_joint = self.finger_joints[i]
            angle = self.convert_motor_value_to_angle(self.current_motor_positions[i], False)
            self.current_angles[base_joint] = angle
            self.current_angles[second_joint] = angle * 0.9

    def finger_position_callback(self, msg: Float32MultiArray):
        """处理接收到的手指位置命令
        
        命令格式：
        - 完整格式：[control_mode(1.0), hand_id(2.0), mode(0.0), thumb_rot, thumb, index, middle, ring, pinky]
        - 简单格式：[thumb_rot, thumb, index, middle, ring, pinky]
        
        Args:
            msg: 包含位置控制命令的消息
        """
        try:
            data = msg.data
            
            # 处理命令格式
            if len(data) > 6:
                command_type = int(data[0])
                if command_type != 1:  # 不是位置控制命令
                    return
                
                # 解析并验证手ID
                hand_id = int(data[1])
                if hand_id != self.selected_hand_id:
                    self.get_logger().warn(f'收到的手ID ({hand_id}) 与配置的手ID ({self.selected_hand_id}) 不匹配，使用配置的手ID')
                    hand_id = self.selected_hand_id
                
                self.current_hand_id = hand_id
                hand_name = "左手" if hand_id == 2 else "右手" if hand_id == 1 else "未知"
                self.get_logger().info(f'使用手ID: {hand_id} ({hand_name})')
                    
                data = list(data[3:])  # 提取位置数据
            elif len(data) != 6:
                return

            current_time = self.get_clock().now()
            
            # 重置抓取状态（新命令开始）
            self.object_grasped = False
            self.grasped_fingers = []
            self.action_should_complete = False
            
            # 智能重置压力停止状态
            for i in range(6):
                # 计算运动方向
                target_pos = float(data[i])
                current_pos = self.current_motor_positions[i]
                
                if target_pos > current_pos:
                    self.movement_directions[i] = 1   # 弯曲方向
                elif target_pos < current_pos:
                    self.movement_directions[i] = -1  # 展开方向
                    # 展开时清除压力停止状态，允许自由展开
                    self.pressure_stopped[i] = False
                else:
                    self.movement_directions[i] = 0   # 无运动
            
            self.get_logger().info(f'======== 新运动命令 ({self.hand_name}) ========')
            
            # 显示运动方向信息
            direction_info = []
            for i in range(6):
                direction = self.movement_directions[i]
                if direction == 1:
                    direction_info.append(f"电机{i}:弯曲")
                elif direction == -1:
                    direction_info.append(f"电机{i}:展开")
                else:
                    direction_info.append(f"电机{i}:停止")
            self.get_logger().info(f'运动方向: {", ".join(direction_info)}')
            
            # 计算每个电机的运动参数
            max_duration = 0.0
            for i in range(6):
                # 保存当前位置
                self.start_motor_positions[i] = self.current_motor_positions[i]
                
                # 设置目标位置
                target_pos = float(data[i])
                self.target_motor_positions[i] = target_pos
                
                # 计算运动时间（基于位置变化量）
                position_change = abs(target_pos - self.current_motor_positions[i])
                base_duration = self.thumb_movement_duration if i < 2 else self.finger_movement_duration
                self.movement_durations[i] = max(0.5, position_change / 1000.0 * base_duration)
                
                # 确定电机类型
                if i == 0:
                    motor_type = "拇指翻转"
                elif i == 1:
                    motor_type = "拇指弯曲"
                else:
                    finger_names = ["食指", "中指", "无名指", "小指"]
                    motor_type = finger_names[i-2]
                
                # 记录详细的运动信息
                pressure_status = "压力停止" if self.pressure_stopped[i] else "正常"
                
                # 更新运动开始时间
                self.movement_start_times[i] = current_time
                self.motors_moving[i] = True
                
                # 记录最长运动时间
                max_duration = max(max_duration, self.movement_durations[i])
            
            self.get_logger().info(f'总预计执行时间: {max_duration:.2f}秒')
            self.get_logger().info('============================')
            
            # 更新目标角度
            self._update_target_angles()
            
        except Exception as e:
            self.get_logger().error(f'处理位置命令时出错: {str(e)}')

    def _update_target_angles(self):
        """更新目标角度并记录日志"""
        # 更新拇指关节目标角度
        self.target_angles['thumb_joint1'] = self.convert_motor_value_to_angle(
            self.target_motor_positions[0], True)
        # self.get_logger().info(f'thumb_joint1 目标位置: {self.target_angles["thumb_joint1"]:.3f}')
        
        # 更新拇指弯曲关节组目标角度
        thumb2_angle, thumb3_angle = self.convert_thumb_angles(self.target_motor_positions[1])
        self.target_angles['thumb_joint2'] = thumb2_angle
        self.target_angles['thumb_joint3'] = thumb3_angle
        
        # 更新其他手指关节目标角度
        for i in range(2, 6):
            base_joint, second_joint = self.finger_joints[i]
            angle = self.convert_motor_value_to_angle(self.target_motor_positions[i], False)
            self.target_angles[base_joint] = angle
            self.target_angles[second_joint] = angle * 0.9

    def pressure_callback(self, msg: JointState):
        """处理压力传感器数据
        
        数据范围：750-3000
        停止阈值：850
        
        当压力超过阈值时，相应手指立即停止运动，模拟真实抓取场景
        
        Args:
            msg: 包含压力传感器数据的消息
        """
        if len(msg.position) < 5:  # 确保数据完整（5个手指的压力值）
            return
        
        # 更新压力数据缓存
        self.last_pressure_values = list(msg.position[:5])
        self.last_pressure_time = self.get_clock().now()
        
        # 记录原始压力值（用于调试）
        raw_pressures = [f"{p:.1f}" for p in msg.position[:5]]
        self.get_logger().info(f'压力监控: [{", ".join(raw_pressures)}] 阈值:{self.pressure_threshold}')
        
        # 更新每个手指的接触状态和压力停止状态
        fingers = ['thumb', 'index', 'middle', 'ring', 'little']
        pressure_status = []
        
        for i, finger in enumerate(fingers):
            current_pressure = msg.position[i]
            # 确保压力值在有效范围内
            current_pressure = max(750, min(current_pressure, 3000))
            
            # 显示每个电机的运动状态
            motor_indices = self.finger_to_motor_map[finger]
            motor_status = []
            for motor_idx in motor_indices:
                moving = "运动中" if self.motors_moving[motor_idx] else "停止"
                stopped = "压力停" if self.pressure_stopped[motor_idx] else "正常"
                motor_status.append(f"M{motor_idx}:{moving}({stopped})")
            
            # 检测接触状态变化
            if current_pressure > self.pressure_threshold:
                pressure_status.append(f"{finger}:{current_pressure:.0f}*[{','.join(motor_status)}]")
                
                if not self.finger_contact[finger]:
                    self.get_logger().warn(f'{finger} 检测到接触！压力值: {current_pressure:.1f} (阈值: {self.pressure_threshold})')
                    self.finger_contact[finger] = True
                    
                    # 停止对应的电机运动（简化版本）
                    if self.enable_pressure_stopping:
                        motor_indices = self.finger_to_motor_map[finger]
                        stopped_motors = []
                        
                        for motor_idx in motor_indices:
                            # 只有在弯曲方向运动时才启用压力停止
                            if (self.motors_moving[motor_idx] and 
                                not self.pressure_stopped[motor_idx] and 
                                hasattr(self, 'movement_directions') and
                                self.movement_directions[motor_idx] == 1):  # 1 = 弯曲方向
                                
                                self.pressure_stopped[motor_idx] = True
                                stopped_motors.append(motor_idx)
                                self.get_logger().warn(f'因 {finger} 压力超过阈值，电机 {motor_idx} 停止弯曲运动！')
                            elif (hasattr(self, 'movement_directions') and 
                                  self.movement_directions[motor_idx] == -1):  # -1 = 展开方向
                                self.get_logger().info(f'{finger} 压力超过阈值，但电机 {motor_idx} 正在展开，继续运动')
                        
                    else:
                        self.get_logger().info(f'压力停止功能已禁用，{finger} 将继续运动')
            else:
                pressure_status.append(f"{finger}:{current_pressure:.0f}[{','.join(motor_status)}]")
                
                if self.finger_contact[finger]:
                    self.get_logger().info(f'{finger} 接触解除，压力值: {current_pressure:.1f}')
                    self.finger_contact[finger] = False
                    
                    # 仅在启用压力停止功能时才处理恢复逻辑
                    if self.enable_pressure_stopping:
                        # 压力恢复正常时，允许电机重新运动（但不自动恢复运动）
                        motor_indices = self.finger_to_motor_map[finger]
                        for motor_idx in motor_indices:
                            if self.pressure_stopped[motor_idx]:
                                self.pressure_stopped[motor_idx] = False
                                self.get_logger().info(f'{finger} 压力恢复正常，电机 {motor_idx} 可以重新运动')
                                self.get_logger().info(f'调试: pressure_stopped[{motor_idx}] = False')
        
        # 显示详细的压力和运动状态
        self.get_logger().info(f'状态: [{", ".join(pressure_status)}]')

    def convert_motor_value_to_angle(self, motor_value: float, is_thumb_rotation: bool = False) -> float:
        """将电机值转换为关节角度
        
        Args:
            motor_value: 电机位置值（0-1000）
            is_thumb_rotation: 是否是拇指翻转关节
        
        Returns:
            float: 关节角度（弧度）
        """
        if is_thumb_rotation:
            # 拇指翻转角度映射（0-90度）
            return self.map_value(
                motor_value,
                0, 1000,
                0.0, 1.57  # 0-90度
            )
        else:
            # 其他关节统一映射（0-90度）
            return self.map_value(
                motor_value,
                0, 1000,
                0.0, 1.57
            )

    def convert_thumb_angles(self, motor_value: float) -> Tuple[float, float]:
        """处理拇指第二和第三关节的角度映射
        
        Args:
            motor_value: 电机位置值（0-1000）
        
        Returns:
            Tuple[float, float]: (拇指第二关节角度, 拇指第三关节角度)
        """
        # 第二关节映射到0-20度
        thumb2_angle = self.map_value(
            motor_value,
            0, 1000,
            0.0, 0.35  # 0-20度
        )
        # 第三关节映射到0-90度
        thumb3_angle = self.map_value(
            motor_value,
            0, 1000,
            0.0, 1.57  # 0-90度
        )
        return thumb2_angle, thumb3_angle

    def map_value(self, value: float, in_min: float, in_max: float, 
                 out_min: float, out_max: float) -> float:
        """将值从一个范围映射到另一个范围
        
        Args:
            value: 输入值
            in_min: 输入范围最小值
            in_max: 输入范围最大值
            out_min: 输出范围最小值
            out_max: 输出范围最大值
        
        Returns:
            float: 映射后的值
        """
        return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def _check_cached_pressure_for_motor(self, motor_idx: int):
        """检查缓存的压力数据是否适用于给定的电机
        
        Args:
            motor_idx: 电机索引
        """
        # 根据电机索引确定对应的手指和压力索引
        finger_pressure_map = {
            0: ('thumb', 0),      # 拇指翻转 -> 拇指压力(索引0)
            1: ('thumb', 0),      # 拇指弯曲 -> 拇指压力(索引0) 
            2: ('index', 1),      # 食指 -> 食指压力(索引1)
            3: ('middle', 2),     # 中指 -> 中指压力(索引2)
            4: ('ring', 3),       # 无名指 -> 无名指压力(索引3)
            5: ('little', 4)      # 小指 -> 小指压力(索引4)
        }
        
        if motor_idx in finger_pressure_map:
            finger_name, pressure_idx = finger_pressure_map[motor_idx]
            current_pressure = self.last_pressure_values[pressure_idx]
            
            # 只有在弯曲方向运动时才检查压力停止
            if (current_pressure > self.pressure_threshold and 
                not self.pressure_stopped[motor_idx] and 
                self.movement_directions[motor_idx] == 1):  # 1 = 弯曲方向
                
                self.pressure_stopped[motor_idx] = True
                self.get_logger().warn(f'缓存检查: {finger_name} 压力{current_pressure:.0f} > {self.pressure_threshold}，电机 {motor_idx} 停止弯曲！')
                
            elif (current_pressure > self.pressure_threshold and 
                  self.movement_directions[motor_idx] == -1):  # -1 = 展开方向
                
                self.get_logger().info(f'缓存检查: {finger_name} 压力{current_pressure:.0f} > {self.pressure_threshold}，但电机 {motor_idx} 正在展开，继续运动')
                
            elif current_pressure <= self.pressure_threshold and self.pressure_stopped[motor_idx]:
                self.pressure_stopped[motor_idx] = False  
                self.get_logger().info(f'缓存检查: {finger_name} 压力{current_pressure:.0f} <= {self.pressure_threshold}，电机 {motor_idx} 恢复运动！')

def main():
    """主函数"""
    rclpy.init()
    node = GripperJointStatePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("正在关闭机械手关节状态发布节点...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
