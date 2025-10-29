/*
 * Copyright (c) 2025 LZRobot. All rights reserved.
 * 灵掌机械手ROS2驱动程序头文件
 *
 * 该文件定义了与灵掌机械手通信和控制的接口。
 * 主要包含：
 * 1. 串口通信的基本操作
 * 2. Modbus RTU协议的实现
 * 3. 机械手控制的高级接口
 */

#ifndef LZ_GRIPPER_DRIVER_HPP_
#define LZ_GRIPPER_DRIVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <vector>
#include <string>
#include <memory>
#include <mutex>

namespace lz_gripper_ros2
{

// 设置位置参数的数量（包括ID和5个手指的位置）
#define NUM_SET_POSITON_PARA 7

/**
 * @brief 命令发送结构体
 * 用于封装发送到机械手的命令数据
 */
typedef struct {
  uint8_t id;                  // 机械手ID
  uint16_t start;             // 起始寄存器地址
  uint16_t size;              // 数据大小
  uint8_t write_num;          // 写入寄存器数量
  uint8_t type;               // 命令类型
  float val[6];               // 控制值数组（位置、速度等）
  uint16_t pos[6];            // 位置值数组（转换后的实际发送值）
  int32_t delaytime;          // 延迟时间
  uint8_t cmd[23];            // Modbus命令数据
  uint8_t response[8];        // Modbus响应数据
  uint8_t openorcloseflag;    // 开合标志
  uint8_t emergencyflag;      // 紧急标志
} send_cmd;

/**
 * @brief 灵掌机械手驱动类
 * 实现与机械手的底层通信和控制功能
 */
class LZGripperDriver
{
public:
  /**
   * @brief 构造函数
   */
  LZGripperDriver();

  /**
   * @brief 析构函数
   */
  ~LZGripperDriver();

  // 底层控制函数
  /**
   * @brief 连接到机械手
   * @param port 串口设备名
   * @param baud_rate 波特率
   * @return 连接是否成功
   */
  bool connect(const std::string& port, int baud_rate);

  /**
   * @brief 断开与机械手的连接
   */
  void disconnect();

  /**
   * @brief 检查是否已连接
   * @return 是否已连接
   */
  bool is_connected() const;

  // 基本控制函数
  /**
   * @brief 写入单个寄存器
   * @param slave_addr 从机地址
   * @param reg_addr 寄存器地址
   * @param value 写入值
   * @return 操作是否成功
   */
  bool write_single_register(uint8_t slave_addr, uint16_t reg_addr, uint16_t value);

  /**
   * @brief 写入多个寄存器
   * @param slave_addr 从机地址
   * @param start_addr 起始地址
   * @param num_registers 寄存器数量
   * @param values 写入值数组
   * @return 操作是否成功
   */
  bool write_multiple_registers(uint8_t slave_addr, uint16_t start_addr, uint16_t num_registers, const std::vector<uint16_t>& values);

  /**
   * @brief 写入多个寄存器（命令方式）
   * @param cmd 命令结构体
   * @return 操作是否成功
   */
  bool write_multiple_registers(send_cmd* cmd);

  /**
   * @brief 读取保持寄存器
   * @param slave_addr 从机地址
   * @param start_addr 起始地址
   * @param num_registers 寄存器数量
   * @param values 读取值数组
   * @return 操作是否成功
   */
  bool read_holding_registers(uint8_t slave_addr, uint16_t start_addr, uint16_t num_registers, std::vector<uint16_t>& values);

  // 高级控制函数
  /**
   * @brief 设置手指位置
   * @param positions 位置数组
   * @return 操作是否成功
   */
  bool set_finger_positions(const std::vector<float>& positions);

  /**
   * @brief 设置手指位置（命令方式）
   * @param cmd 命令结构体
   * @return 操作是否成功
   */
  bool set_finger_positions(send_cmd* cmd);

  /**
   * @brief 设置手指速度
   * @param cmd 速度命令结构体
   * @return 操作是否成功
   */
  bool set_finger_velocity(send_cmd* cmd);

  /**
   * @brief 获取手指压力值
   * @param positions 存储压力值的数组
   * @return 操作是否成功
   */
  bool get_finger_pressure(std::vector<float>& positions);

  /**
   * @brief 获取指定ID机械手的压力值
   * @param positions 存储压力值的数组
   * @param id 机械手ID
   * @return 操作是否成功
   */
  bool get_finger_pressure(std::vector<float>& positions, uint8_t id);

  /**
   * @brief 设置机械手状态
   * @param param 参数数组（包含ID和开合状态）
   * @return 操作是否成功
   */
  bool set_gripper_state(const std::vector<int32_t>& param);

  /**
   * @brief 获取机械手状态
   * @param is_closed 存储机械手是否闭合的状态
   * @return 操作是否成功
   */
  bool get_gripper_state(bool& is_closed);

private:
  // Modbus RTU 协议相关函数
  /**
   * @brief 计算Modbus CRC校验码
   * @param data 数据
   * @param len 数据长度
   * @return CRC校验码
   */
  uint16_t modbus_crc(const uint8_t* data, int len);

  /**
   * @brief 发送Modbus请求
   * @param request 请求数据
   * @param len 数据长度
   * @return 发送是否成功
   */
  bool send_request(const uint8_t* request, int len);

  /**
   * @brief 接收Modbus响应
   * @param response 响应数据缓冲区
   * @param max_len 最大接收长度
   * @param timeout_ms 超时时间（毫秒）
   * @return 接收到的字节数，失败返回-1
   */
  int receive_response(uint8_t* response, int max_len, int timeout_ms);

  // 私有成员变量
  int serial_fd_;             // 串口文件描述符
  std::mutex mutex_;          // 互斥锁，用于保护串口访问
  bool is_connected_;         // 连接状态标志
  
  // 常量定义
  static constexpr uint8_t SLAVE_ADDR = 2;    // 默认从机地址
  static constexpr int NUM_FINGERS = 5;        // 手指数量

public:
  // 位置限制常量
  static constexpr float MAX_POSITION = 1000.0f;  // 最大位置值
  static constexpr float MIN_POSITION = 0.0f;     // 最小位置值
};

} // namespace lz_gripper_ros2

#endif // LZ_GRIPPER_DRIVER_HPP_ 