# LZ Gripper ROS2 Package

灵掌灵巧手驱动接口说明，包含底层驱动接口以及高层控制接口.

## Features

- 灵巧手 Modbus RTU 底层通信
- 灵巧手高层控制接口以以及手掌状态控制
- ROS2 话题
- 可以配置通信接口以及参数设

## Dependencies

- ROS2 完整安装(tested with jazzy)
- serial_driver

## Installation

1. 下载当前代码到当前分支 ROS2 workspace:
```bash
cd ~/ros2_ws/
git clone <repository_url> lz_gripper_ros2
```

2. 编译工具包:
```bash
cd lz_gripper_ros2
colcon build --packages--up--to syml 
```

3. 初始化工作空间:
```bash
source install/setup.bash
```

## 使用方式

### 启动灵掌灵巧手节点

```bash
ros2 launch lz_gripper_ros2 lz_gripper.launch.py
```

可设置选项参数:
- `port`: 串口名称 (默认: /dev/ttyUSB0)
- `baud_rate`: 波特率 (默认: 115200)
- `publish_rate`: 关节状态刷新率 (默认: 10.0 Hz)

自定义参数示例:
```bash
ros2 launch lz_gripper_ros2 lz_gripper.launch.py port:=/dev/ttyUSB0 baud_rate:=115200
ros2 launch lz_gripper_ros2 lz_gripper.launch.py port:=/dev/ttyCH341USB0 baud_rate:=115200
```

### 话题

#### Subscribed Topics

- `/finger_positions` (std_msgs/Float32MultiArray)
  - Set individual finger positions (0-1000)
  - Array size must be 6(one value per finger)

- `/gripper_command` (std_msgs/Bool)
  - true: Close gripper
  - false: Open gripper

- `/write_register` (std_msgs/Float32MultiArray)
  - Finger_Positions：手指关节位置（0-1000）
  - gripper_Command：灵巧手握紧张开（0 or 1）
  - Finger_Velocity：手指运动速度设定（0-1000）
  - Finger_Effort：手指力度设定（0-1000）

#### Published Topics

- `/lv_gripper_states` (sensor_msgs/JointState)
  - Current position of each finger
  - Joint names: finger1, finger2, finger3, finger4, finger5

- `/gripper_state` (std_msgs/Bool)
  - true: Gripper is closed
  - false: Gripper is open

### 获取灵巧手压力传感器状态

数据范围750–3000（有效值）

```bash
ros2 topic echo /lv_gripper_states 
```
### Example Usage in Python

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

Finger_Positions=1
Gripper_Command=1
Finger_Velocity=3
Finger_Effort=4
Read_Finger_Positions=5
Read_gripper_Command=6
Read_Finger_Velocity=7
Read_Finger_Effort=8
Left_Hand=2
Right_Hand=1
Emergency=1
Normal=0

class GripperController(Node):
    def __init__(self):
        super().__init__('gripper_controller')

        self.register_pub=self.create_publisher(
            Float32MultiArray, 'write_register', 10)
    def set_register(self,data):
        msg=Float32MultiArray()
        msg.data=data
        self.register_pub.publish(msg)

def main():
    rclpy.init()
    controller = GripperController()
    
    # Example: Set specific finger positions

    controller.set_register([Finger_Effort,Left_Hand,Normal,1,1000,1000,1000,1000,1000,1000])
    controller.set_register([Finger_Velocity,Left_Hand,Normal,1,1000,1000,1000,1000,1000,1000])
    #数组各部分含义：功能，灵巧手id（左手2,右手1），控制动作持续时间（单位1ms）,大拇指翻转幅度（最小0,最大1000），
#               大拇指弯曲幅度，食指弯曲幅度，中指弯曲幅度，无名指弯曲幅度，小拇指弯曲幅度
    controller.set_register([Finger_Positions,Left_Hand,544,500, 500, 500, 300, 300,300])
    controller.set_register([Finger_Positions,Left_Hand,530,100, 100, 100, 300, 300,300])
    #
    controller.set_register([Finger_Velocity,Left_Hand,Normal,1,400,400,400,400,400,400])
    controller.set_register([Finger_Positions,Left_Hand,1030,500, 500, 500, 500, 500,500])
    controller.set_register([Finger_Positions,Left_Hand,630,100, 100, 100, 100, 100,100])

    controller.set_register([Finger_Velocity,Left_Hand,Normal,1,600,600,600,600,600,600])
    controller.set_register([Finger_Effort,Left_Hand,Normal,1,1000,1000,1000,1000,1000,1000])
    controller.set_register([Finger_Positions,Left_Hand,1030,400, 400, 400, 400, 400,400])
    controller.set_register([Finger_Positions,Left_Hand,1030,0, 0, 0, 0, 0,0])
    controller.set_register([Finger_Effort,Left_Hand,Normal,1,500,500,500,500,500,500])
    controller.set_register([Finger_Positions,Left_Hand,1030,400, 400, 400, 400, 400,400])
    
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 代码说明

使用write_register对灵巧手进行控制操作，可以确保灵巧手按程序的顺序执行。使用单独的功能话题通信，需要注意的是，存在不同类型的控制指令时，由于线程的优先级问题，可能会导致灵巧手的动作与程序顺序不一致。

set_register() 详细说明

|第1位|第2位|第3位|第4位|第5位|第6位|第7位|第8位|第9位|第10位|其他说明|
|---|---|---|---|---|---|---|---|---|---|---|
|Finger_Positions|Left_Hand(Right_Hand)|运行时间(单位ms)|大拇指翻转幅度（最小0,最大1000）|大拇指弯曲幅度|食指弯曲幅度|中指弯曲幅度|无名指弯曲幅度|小拇指弯曲幅度||设置各个指关节弯曲幅度|
|Gripper_Command|Left_Hand(Right_Hand)|运行时间(单位ms)|0 or 1|||||||设置灵巧手握紧松开|
|Finger_Velocity|Left_Hand(Right_Hand)|紧急或正常|运行时间(单位ms)|大拇指翻转速度（最小0,最大1000）|大拇指运动速度|食指运动速度|中指运动速度|无名指运动速度|小拇指运动速度|设置各个指关节运动速度|
|Finger_Effort|Left_Hand(Right_Hand)|紧急或正常|运行时间(单位ms)|大拇指翻转力度（最小0,最大1000）|大拇指力度|食指力度|中指力度|无名指力度|小拇指力度|设置各个指关节力度|

## License

This package is licensed under the Apache License 2.0. See the LICENSE file for details.

Copyright (c) 2025 LZRobot. All rights reserved.

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request. For major changes, please open an issue first to discuss what you would like to change. 