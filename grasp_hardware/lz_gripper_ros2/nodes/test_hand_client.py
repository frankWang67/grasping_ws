#!/usr/bin/env python3
"""
机械手测试客户端

功能：
- 提供交互式手势控制界面
- 支持预定义位置、手动输入和快速测试序列
- 智能错误检测（区分展开/弯曲动作的失败原因）
- 实时进度和压力反馈
- 支持左右手选择控制
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import time
import threading
import random
from array import array
from lz_gripper_ros2.action import HandCommand

# 控制常量
class ControlModes:
    FINGER_POSITIONS = 1.0  # 位置控制模式
    NORMAL = 0.0            # 正常模式

class HandIDs:
    LEFT_HAND = 2.0         # 左手
    RIGHT_HAND = 1.0        # 右手

class HandTestClient(Node):
    """机械手测试客户端节点"""
    
    def __init__(self):
        super().__init__('hand_test_client')
        
        # 声明参数
        self.declare_parameter('hand_side', 'left')
        
        # 读取手边参数并设置手ID
        hand_side = self.get_parameter('hand_side').get_parameter_value().string_value
        if hand_side.lower() == 'left':
            self.selected_hand_id = HandIDs.LEFT_HAND
            self.hand_name = "左手"
        elif hand_side.lower() == 'right':
            self.selected_hand_id = HandIDs.RIGHT_HAND
            self.hand_name = "右手"
        else:
            self.get_logger().warn(f'未知的手边参数: {hand_side}，默认使用左手')
            self.selected_hand_id = HandIDs.LEFT_HAND
            self.hand_name = "左手"
            
        self.get_logger().info(f'测试客户端配置: {self.hand_name} (ID: {int(self.selected_hand_id)})')
        
        self._stop_flag = threading.Event()
        self._setup_action_client()

    def _setup_action_client(self):
        """设置Action客户端"""
        self._action_client = ActionClient(self, HandCommand, 'hand_control')

        self.get_logger().info("等待动作服务器...")
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("动作服务器连接超时！")
            return

        self.get_logger().info("动作服务器已就绪")

    def send_goal(self, data):
        """发送目标位置并等待结果
        
        Args:
            data: 控制命令数据
            
        Returns:
            bool: 执行是否成功
        """
        try:
            # 创建并发送目标
            goal_msg = HandCommand.Goal()
            goal_msg.data = array('f', data)
            
            self.get_logger().info(f'发送控制命令: {goal_msg.data}')
            
            # 发送目标
            future = self._action_client.send_goal_async(
                goal_msg, feedback_callback=self._feedback_callback)
        
            # 等待目标被接受
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if not future.done():
                self.get_logger().error("发送目标超时")
                return False
        
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error("目标被拒绝")
                return False
            
            self.get_logger().info("目标被接受，机械手开始运动...")
            
            # 等待执行结果
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=60.0)
            
            # 处理执行结果
            return self._handle_execution_result(result_future)
                
        except Exception as e:
            self.get_logger().error(f'发送目标时发生错误: {str(e)}')
            return False

    def _handle_execution_result(self, result_future):
        """处理执行结果"""
        if not result_future.done():
            self.get_logger().error("❌ 动作执行超时，请检查硬件连接或延长超时时间")
            return False
        
        result = result_future.result()
        if result.result.success:
            self.get_logger().info(f"✅ 运动完成: {result.result.message}")
            if result.result.pressures:
                pressure_str = ", ".join([f"{p:.0f}" for p in result.result.pressures])
                self.get_logger().info(f"最终压力值: [{pressure_str}]")
            return True
        else:
            self.get_logger().error(f"❌ 运动失败: {result.result.message}")
            return False

    def _feedback_callback(self, feedback_msg):
        """反馈回调函数"""
        feedback = feedback_msg.feedback
        progress = feedback.completion * 100
        
        self.get_logger().info(f'运动进度: {progress:.1f}%')
        
        if feedback.pressures:
            pressure_str = ", ".join([f"{p:.0f}" for p in feedback.pressures])
            self.get_logger().info(f'当前压力值: [{pressure_str}]')

    def get_predefined_positions(self):
        """获取预定义手势位置"""
        return {
            "1": {
                "name": "完全张开",
                "positions": [0, 0, 0, 0, 0, 0],
                "description": "所有手指完全伸直"
            },
            "2": {
                "name": "自然握拳", 
                "positions": [500, 800, 800, 800, 800, 800],
                "description": "模拟自然握拳姿态"
            },
            "3": {
                "name": "精细抓取",
                "positions": [700, 600, 600, 600, 400, 400],
                "description": "适合抓取小物体"
            },
            "4": {
                "name": "OK手势",
                "positions": [800, 800, 800, 0, 0, 0],
                "description": "拇指和食指形成圆圈"
            },
            "5": {
                "name": "点赞手势",
                "positions": [0, 0, 800, 800, 800, 800],
                "description": "拇指向上，其他手指弯曲"
            },
            "6": {
                "name": "握手动作",
                "positions": [700, 400, 200, 200, 300, 600],
                "description": "拇指翻转对握，其他手指轻微弯曲"
            },
            "7": {
                "name": "持续性随机动作",
                "positions": "random_continuous",
                "description": "持续执行随机手势"
            }
        }

    def interactive_control(self):
        """交互式控制主循环"""
        self._print_welcome_info()
        
        while not self._stop_flag.is_set():
            try:
                choice = self._get_user_choice()
                
                if choice == '4':
                    self._handle_exit()
                    break
                elif choice == '1':
                    self._handle_predefined_positions()
                elif choice == '2':
                    self._handle_manual_input()
                elif choice == '3':
                    self._handle_test_sequence()
                else:
                    print("无效选择，请重试")
                    
            except KeyboardInterrupt:
                self._handle_interrupt()
                break
            except Exception as e:
                self.get_logger().error(f'发生错误: {str(e)}')

    def _print_welcome_info(self):
        """打印欢迎信息"""
        print("\n" + "="*60)
        print(f"机械手位置控制测试客户端 - {self.hand_name}")
        print("="*60)
        print("功能说明:")
        print("• 支持预定义位置快速测试")
        print("• 支持手动输入各手指位置值")
        print("• 支持快速测试序列")
        print("• 位置值范围: 0-1000 (0=完全伸直, 1000=完全弯曲)")
        print(f"• 当前控制: {self.hand_name} (ID: {int(self.selected_hand_id)})")
        print("="*60)

    def _get_user_choice(self):
        """获取用户选择"""
        print("\n" + "-"*50)
        print(f"请选择控制方式 ({self.hand_name}):")
        print("1. 使用预定义位置")
        print("2. 手动输入各手指位置")
        print("3. 快速测试序列")
        print("4. 退出程序")
        print("-"*50)
        return input("请选择 (1-4): ").strip()

    def _handle_exit(self):
        """处理退出"""
        print("退出程序...")
        self._stop_flag.set()

    def _handle_interrupt(self):
        """处理中断"""
        print("\n程序被用户中断")
        self._stop_flag.set()

    def _handle_predefined_positions(self):
        """处理预定义位置选择"""
        presets = self.get_predefined_positions()
        
        print(f"\n预定义位置 ({self.hand_name}):")
        for key, preset in presets.items():
            print(f"{key}. {preset['name']} - {preset['description']}")
        
        preset_choice = input("请选择预定义位置 (1-7): ").strip()
        
        if preset_choice in presets:
            preset = presets[preset_choice]
            
            # 特殊处理持续性随机动作
            if preset['positions'] == "random_continuous":
                self._handle_continuous_random_movement()
            else:
                print(f"\n执行: {preset['name']} ({self.hand_name})")
                print(f"位置值: {preset['positions']}")
                
                # 构建并发送命令
                command = self._build_command(preset['positions'])
                success = self.send_goal(command)
                
                # 显示结果
                result_text = "执行成功" if success else "执行失败"
                print(f"{result_text} {preset['name']} {result_text}")
        else:
            print("无效选择")

    def _handle_manual_input(self):
        """处理手动输入"""
        print(f"\n请输入各手指的位置值 (0-1000) - {self.hand_name}:")
        
        finger_names = ["拇指翻转", "拇指弯曲", "食指", "中指", "无名指", "小指"]
        positions = []
        
        # 获取每个手指的位置值
        for finger_name in finger_names:
            while True:
                try:
                    value = input(f"{finger_name} (0-1000): ").strip()
                    if value == "":
                        value = 0
                    else:
                        value = float(value)
                    
                    if 0 <= value <= 1000:
                        positions.append(value)
                        break
                    else:
                        print("请输入0-1000之间的数值")
                except ValueError:
                    print("请输入有效的数字")
        
        # 确认执行
        print(f"\n确认位置 ({self.hand_name}): {positions}")
        confirm = input("确认执行? (y/N): ").strip().lower()
        
        if confirm in ['y', 'yes']:
            command = self._build_command(positions)
            success = self.send_goal(command)
            
            result_text = "执行成功" if success else "执行失败"
            print(f"{result_text} 位置控制{result_text}")
        else:
            print("取消执行")

    def _handle_test_sequence(self):
        """处理测试序列"""
        print(f"\n开始快速测试序列 ({self.hand_name})...")
        
        test_positions = [
            ("完全张开", [0, 0, 0, 0, 0, 0]),
            ("握拳", [500, 800, 800, 800, 800, 800]),
            ("张开", [0, 0, 0, 0, 0, 0]),
            ("精细抓取", [700, 600, 600, 600, 400, 400]),
            ("恢复张开", [0, 0, 0, 0, 0, 0])
        ]
        
        for i, (name, positions) in enumerate(test_positions, 1):
            if self._stop_flag.is_set():
                break
                
            print(f"\n 步骤 {i}/5: {name} ({self.hand_name})")
            
            command = self._build_command(positions)
            success = self.send_goal(command)
            
            result_text = "完成" if success else "失败"
            print(f"{result_text} {name} {result_text}")
            
            if not success:
                break
            
            if i < len(test_positions):
                print("等待2秒...")
                time.sleep(2)
        
        print(f"\n测试序列完成! ({self.hand_name})")

    def _build_command(self, positions):
        """构建控制命令"""
        return [
            ControlModes.FINGER_POSITIONS,  # 位置控制模式
            self.selected_hand_id,          # 使用选定的手ID
            ControlModes.NORMAL,            # 正常模式
        ] + positions

    def _handle_continuous_random_movement(self):
        """处理持续性随机动作"""
        print(f"\n开始持续性随机动作 ({self.hand_name})...")
        print("特点：大拇指保护性弯曲，防止碰撞")
        print("按 Ctrl+C 或输入 'q' 停止")
        
        # 设置停止事件
        stop_random = threading.Event()
        
        # 创建输入监听线程
        def input_listener():
            try:
                while not stop_random.is_set():
                    user_input = input().strip().lower()
                    if user_input == 'q' or user_input == 'quit':
                        stop_random.set()
                        break
            except:
                pass
        
        input_thread = threading.Thread(target=input_listener, daemon=True)
        input_thread.start()
        
        try:
            movement_count = 0
            while not stop_random.is_set() and not self._stop_flag.is_set():
                movement_count += 1
                
                # 生成随机位置
                random_positions = self._generate_safe_random_positions()
                
                print(f"\n随机动作 #{movement_count} ({self.hand_name})")
                print(f"目标位置: {random_positions}")
                
                # 执行动作
                command = self._build_command(random_positions)
                success = self.send_goal(command)
                
                if not success:
                    print("动作执行失败，停止随机动作")
                    break
                
                # 检查是否需要停止
                if stop_random.wait(timeout=2.0):  # 2秒间隔
                    break
                    
        except KeyboardInterrupt:
            print("\n收到中断信号，停止随机动作")
        finally:
            stop_random.set()
            print(f"\n持续性随机动作已停止 ({self.hand_name})")
            
            # 回到张开状态
            print("恢复到张开状态...")
            open_command = self._build_command([0, 0, 0, 0, 0, 0])
            self.send_goal(open_command)

    def _generate_safe_random_positions(self):
        """生成安全的随机位置，大拇指弯曲幅度较小"""
        positions = []
        
        # 拇指翻转：适中范围，避免极端位置
        thumb_rotation = random.randint(0,1000)
        positions.append(thumb_rotation)
        
        # 拇指弯曲：保护性范围，防止碰撞
        thumb_bend = random.randint(0, 500)  # 很小的弯曲范围
        positions.append(thumb_bend)
        
        # 其他四指：较大范围的随机动作
        for _ in range(4):  # 食指、中指、无名指、小指
            finger_pos = random.randint(0, 800)
            positions.append(finger_pos)
        
        return positions

def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    try:
        client = HandTestClient()
        
        # 在单独线程中运行交互控制
        control_thread = threading.Thread(
            target=client.interactive_control,
            daemon=True
        )
        control_thread.start()
        
        # 主线程运行ROS2事件循环
        try:
            rclpy.spin(client)
        except KeyboardInterrupt:
            client.get_logger().info('操作被用户中断')
            client._stop_flag.set()
        
        # 等待控制线程结束
        control_thread.join(timeout=1.0)
        
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main() 