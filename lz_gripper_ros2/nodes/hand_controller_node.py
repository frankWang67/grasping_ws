#!/usr/bin/env python3
"""
机械手控制器节点

功能：
- 提供HandCommand Action服务
- 计算智能运动时间（基于位置变化量）
- 实时反馈运动进度和压力值
- 支持动作成功/失败状态反馈
- 支持左右手选择控制
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Float32MultiArray
from lz_gripper_ros2.srv import ServerMsg
from lz_gripper_ros2.action import HandCommand
import time
import asyncio

# 服务命令常量
class Commands:
    READ_FINGER_PRESSURE = 5  # 读取手指压力值
    
class HandIDs:
    LEFT_HAND = 2             # 左手ID
    RIGHT_HAND = 1            # 右手ID

class HandControllerNode(Node):
    """机械手控制器节点"""
    
    def __init__(self):
        super().__init__('hand_controller')
        
        # 声明参数
        self.declare_parameter('hand_side', 'left')
        
        # 读取手边参数并设置手ID
        hand_side = self.get_parameter('hand_side').get_parameter_value().string_value
        if hand_side.lower() == 'left':
            self.selected_hand_id = HandIDs.LEFT_HAND
            hand_name = "左手"
        elif hand_side.lower() == 'right':
            self.selected_hand_id = HandIDs.RIGHT_HAND
            hand_name = "右手"
        else:
            self.get_logger().warn(f'未知的手边参数: {hand_side}，默认使用左手')
            self.selected_hand_id = HandIDs.LEFT_HAND
            hand_name = "左手"
            
        self.get_logger().info(f'手控制器配置: {hand_name} (ID: {self.selected_hand_id})')
        
        # 运动参数配置
        self.thumb_movement_duration = 4.0   # 拇指满行程时间（秒）
        self.finger_movement_duration = 8.0  # 其他手指满行程时间（秒）
        self.min_movement_threshold = 10     # 最小运动阈值
        
        # 状态变量
        self.current_positions = [0.0] * 6   # 6个电机的当前位置
        self.current_hand_id = self.selected_hand_id  # 使用选定的手ID
        
        self._setup_communication()
        self.get_logger().info(f'手控制器节点已启动，控制{hand_name}')

    def _setup_communication(self):
        """设置ROS2通信组件"""
        # 使用ReentrantCallbackGroup允许并发执行
        self.callback_group = ReentrantCallbackGroup()
        
        # 创建Action服务器
        self._action_server = ActionServer(
            self,
            HandCommand,
            'hand_control',
            self.execute_callback,
            callback_group=self.callback_group
        )
        
        # 创建发布者
        self.register_pub = self.create_publisher(
            Float32MultiArray, 'write_register', 10)
        
        # 创建服务客户端
        self.cli = self.create_client(
            ServerMsg, 'lz_gripper_service', callback_group=self.callback_group)
        
        self._wait_for_service()
        self.req = ServerMsg.Request()

    def _wait_for_service(self):
        """等待服务可用"""
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待服务可用...')

    def calculate_movement_duration(self, target_positions):
        """计算智能运动时间
        
        Args:
            target_positions: 目标位置列表 [0-1000]
            
        Returns:
            float: 运动持续时间（秒）
        """
        max_duration = 0.0
        has_significant_movement = False
        
        self.get_logger().info(f'当前位置: {[f"{p:.0f}" for p in self.current_positions]}')
        self.get_logger().info(f'目标位置: {[f"{p:.0f}" for p in target_positions]}')
        
        for i, target_pos in enumerate(target_positions):
            # 计算运动距离
            position_change = abs(target_pos - self.current_positions[i])
            
            # 确定运动参数
            motor_info = self._get_motor_info(i)
            base_duration = motor_info['duration']
            motor_name = motor_info['name']
            
            # 计算运动时间
            if position_change > self.min_movement_threshold:
                # 有明显运动：按比例计算时间
                actual_duration = max(0.5, position_change / 1000.0 * base_duration)
                has_significant_movement = True
            else:
                # 微小变化：设置短时间
                actual_duration = 0.1
            
            self.get_logger().info(
                f'电机{i} ({motor_name}): {self.current_positions[i]:.0f}→{target_pos:.0f} '
                f'(距离:{position_change:.0f}, 时间:{actual_duration:.1f}s)')
            
            max_duration = max(max_duration, actual_duration)
        
        # 处理无明显运动的情况
        if not has_significant_movement:
            max_duration = 0.5
            self.get_logger().info('无明显运动，设置快速完成模式')
        
        self.get_logger().info(f'总运动时间: {max_duration:.2f}秒')
        return max_duration

    def _get_motor_info(self, motor_index):
        """获取电机信息"""
        if motor_index < 2:
            return {
                'duration': self.thumb_movement_duration,
                'name': '拇指翻转' if motor_index == 0 else '拇指弯曲'
            }
        else:
            finger_names = ['食指', '中指', '无名指', '小指']
            return {
                'duration': self.finger_movement_duration,
                'name': finger_names[motor_index - 2]
            }

    async def execute_callback(self, goal_handle):
        """执行动作回调"""
        self.get_logger().info('收到新动作请求')
        
        feedback_msg = HandCommand.Feedback()
        result = HandCommand.Result()
        
        try:
            # 解析目标位置
            target_positions = self._parse_target_positions(goal_handle.request.data)
            
            # 计算运动时间
            duration = self.calculate_movement_duration(target_positions)
            
            # 发送控制命令
            await self._send_control_command(goal_handle.request.data)
            self.get_logger().info(f'控制命令已发送，预计执行时间: {duration:.2f}秒')
            
            # 执行运动并提供反馈
            success = await self._execute_movement_with_feedback(
                goal_handle, feedback_msg, duration)
            
            if success:
                # 更新当前位置并立即返回成功结果
                self.current_positions = list(target_positions)
                
                result.success = True
                result.message = "动作执行成功"
                result.pressures = []  # 跳过最终压力读取，避免延迟
                
                self.get_logger().info('运动成功完成')
                goal_handle.succeed()
            else:
                raise Exception("运动执行失败")
                
        except Exception as e:
            self.get_logger().error(f'动作执行失败: {str(e)}')
            result.success = False
            result.message = f"动作执行失败: {str(e)}"
            goal_handle.abort()
        
        return result

    def _parse_target_positions(self, request_data):
        """解析目标位置数据并提取手ID"""
        if len(request_data) >= 9:
            # 完整格式：[控制模式, 手ID, 模式, 位置1, 位置2, ...]
            hand_id = int(request_data[1])
            
            # 验证手ID是否与配置匹配
            if hand_id != self.selected_hand_id:
                self.get_logger().warn(f'收到的手ID ({hand_id}) 与配置的手ID ({self.selected_hand_id}) 不匹配，使用配置的手ID')
                hand_id = self.selected_hand_id
            
            self.current_hand_id = hand_id  # 更新当前手ID
            hand_name = "左手" if hand_id == 2 else "右手" if hand_id == 1 else "未知"
            self.get_logger().info(f'使用手ID: {hand_id} ({hand_name})')
            return request_data[3:9]  # 跳过前3个控制参数
        elif len(request_data) == 6:
            # 简单格式：直接使用位置数据，使用配置的手ID
            self.current_hand_id = self.selected_hand_id
            hand_name = "左手" if self.selected_hand_id == 2 else "右手"
            self.get_logger().info(f'使用配置的手ID: {self.selected_hand_id} ({hand_name})')
            return request_data
        else:
            raise ValueError(f"无效的数据长度: {len(request_data)}")

    async def _send_control_command(self, data):
        """发送控制命令到硬件"""
        msg = Float32MultiArray()
        msg.data = [float(f) for f in data]
        self.register_pub.publish(msg)

    async def _execute_movement_with_feedback(self, goal_handle, feedback_msg, duration):
        """执行运动并提供实时反馈"""
        start_time = self.get_clock().now()
        feedback_interval = 0.2  # 200ms反馈间隔
        last_log_time = 0.0
        last_pressure_time = 0.0
        pressure_interval = 2.0  # 2秒读取一次压力值，大幅降低频率
        cached_pressure_values = []  # 缓存压力值
        
        while True:
            elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
            
            # 检查是否完成
            if elapsed >= duration:
                break
            
            # 大幅降低压力读取频率，避免频繁超时
            pressure_values = cached_pressure_values
            if elapsed - last_pressure_time >= pressure_interval:
                try:
                    new_pressure = await self._get_finger_pressure()
                    if new_pressure:
                        cached_pressure_values = new_pressure
                        pressure_values = new_pressure
                    last_pressure_time = elapsed
                except Exception as e:
                    self.get_logger().debug(f'获取压力值异常: {str(e)}')
                    # 继续使用缓存值，不阻塞进度
                
            progress_percentage = self._calculate_progress(elapsed, duration)
            
            # 更新反馈消息
            feedback_msg.pressures = pressure_values
            feedback_msg.completion = progress_percentage / 100.0
            
            # 定期记录日志
            if elapsed - last_log_time >= 1.0:
                self._log_progress(pressure_values, progress_percentage, elapsed, duration)
                last_log_time = elapsed
            
            # 发送反馈
            goal_handle.publish_feedback(feedback_msg)
            
            # 检查取消请求
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('动作被取消')
                return False
            
            time.sleep(feedback_interval)
        
        return True

    def _calculate_progress(self, elapsed, duration):
        """计算运动进度百分比"""
        if duration > 0:
            return min(100.0, (elapsed / duration) * 100.0)
        return 100.0

    def _log_progress(self, pressure_values, progress, elapsed, duration):
        """记录进度日志"""
        if pressure_values:
            pressure_str = ", ".join([f"{p:.0f}" for p in pressure_values])
            self.get_logger().info(f'压力值: [{pressure_str}]')
        
        self.get_logger().info(
            f'进度: {progress:.1f}% ({elapsed:.1f}s/{duration:.1f}s)')

    async def _get_finger_pressure(self):
        """获取手指压力值"""
        try:
            self.req.messageid = Commands.READ_FINGER_PRESSURE
            self.req.id = self.current_hand_id  # 使用当前手ID
            self.req.flag = 0
            self.req.delay = 0
            
            future = self.cli.call_async(self.req)
            
            # 添加超时保护：1秒超时
            try:
                response = await asyncio.wait_for(future, timeout=1.0)
                
                if response and hasattr(response, 'pressure'):
                    return [float(p) for p in response.pressure]
            except asyncio.TimeoutError:
                self.get_logger().debug('获取压力值超时')
                return None
            
        except Exception as e:
            self.get_logger().debug(f'获取压力值失败: {str(e)}')
        
        return None

def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    node = HandControllerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 