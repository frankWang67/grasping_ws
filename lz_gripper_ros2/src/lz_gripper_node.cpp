/*
 * Copyright (c) 2025 LZRobot. All rights reserved.
 * 灵掌机械手ROS2节点实现文件
 * 
 * 该文件实现了ROS2节点，用于控制灵掌机械手。
 * 主要功能：
 * 1. 提供ROS2话题接口，用于控制机械手和获取状态
 * 2. 管理串口通信和命令队列
 * 3. 处理传感器数据和状态反馈
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "lz_gripper_ros2/lz_gripper_driver.hpp"
#include <chrono>
#include <thread>
#include <iostream>
#include <string.h>
#include "lz_gripper_ros2/srv/server_msg.hpp"

using namespace std::chrono_literals;
using namespace lz_gripper_ros2;

// 常量定义
#define QUEUE_NUM 64                  // 命令队列大小
#define POSITION_SUB_CALLBACK  1      // 位置控制回调标识
#define GRIPPER_COMMAND_CALLBACK 2    // 机械手命令回调标识
#define VELOCITY_SUB_CALLBACK 3       // 速度控制回调标识
#define EFFORT_SUB_CALLBACK 4         // 力控制回调标识

// 命令类型定义
#define CMD_TYPE_SET_POSITION 0       // 设置位置命令
#define CMD_TYPE_SET_VELOCITY 1       // 设置速度命令
#define CMD_TYPE_SET_EFFORT    2      // 设置力控制命令
#define CMD_TYPE_GET_PRESSURE  5      // 获取压力值命令

/**
 * @brief 接收数据的缓存结构
 */
typedef struct {
  int32_t total_count;               // 总计数
  int32_t error_count;               // 错误计数
  int8_t overflow;                   // 溢出标志
  int32_t total_max_count;           // 最大总计数
  int32_t error_max_count;           // 最大错误计数
  int16_t value[100][5];            // 数据缓存数组（100个时间点，每个时间点5个手指的数据）
  int16_t head;                      // 缓存头指针
  int16_t tail;                      // 缓存尾指针
} receive_data;

/**
 * @brief 灵掌机械手ROS2节点类
 */
class LZGripperNode : public rclcpp::Node {
public:
  /**
   * @brief 构造函数
   * 初始化ROS2节点，设置参数，创建发布者、订阅者和服务
   */
  LZGripperNode() : Node("lz_gripper_node") {
    // 声明和获取参数
    this->declare_parameter("port", "/dev/ttyUSB0");       // 串口设备名
    this->declare_parameter("baud_rate", 115200);          // 波特率
    this->declare_parameter("publish_rate", 10.0);         // 发布频率

    std::string port = this->get_parameter("port").as_string();
    int baud_rate = this->get_parameter("baud_rate").as_int();
    publish_rate = this->get_parameter("publish_rate").as_double();
    RCLCPP_INFO(this->get_logger(), "发布频率: %f", publish_rate);

    // 创建驱动实例
    driver_ = std::make_unique<lz_gripper_ros2::LZGripperDriver>();

    emergency_execution = 0;
    // 连接到机械手
    if (!driver_->connect(port, baud_rate)) {
      RCLCPP_ERROR(this->get_logger(), "无法连接到机械手，端口: %s", port.c_str());
      return;
    }
    RCLCPP_INFO(this->get_logger(), "成功连接到机械手，端口: %s", port.c_str());

    // 创建发布者
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "lz_gripper_states", 10);    // 发布关节状态
    gripper_state_pub_ = this->create_publisher<std_msgs::msg::Bool>(
        "gripper_state", 10);        // 发布机械手状态

    // 创建订阅者
    position_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "finger_positions", 10,       // 订阅手指位置命令
        std::bind(&LZGripperNode::command_insert_callback, this, std::placeholders::_1));

    gripper_command_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "gripper_command", 10,        // 订阅机械手命令
        std::bind(&LZGripperNode::command_insert_callback, this, std::placeholders::_1));

    velocity_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "finger_velocity", 10,        // 订阅手指速度命令
        std::bind(&LZGripperNode::command_insert_callback, this, std::placeholders::_1));

    register_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "write_register", 10,         // 订阅寄存器写入命令
        std::bind(&LZGripperNode::command_insert_callback, this, std::placeholders::_1));

    // 创建服务
    service_ = create_service<lz_gripper_ros2::srv::ServerMsg>(
        "lz_gripper_service",         // 提供机械手控制服务
        [this](
            const std::shared_ptr<lz_gripper_ros2::srv::ServerMsg::Request> request,
            std::shared_ptr<lz_gripper_ros2::srv::ServerMsg::Response> response
        ) {
            this->service_handle(request, response);
        }
    );

    // 初始化变量
    python_delay = 0;
    cmd_send_queue_head = 0;
    cmd_send_queue_tail = 0;
    callback_lock = 0;
    
    // 清空命令队列和接收数据缓存
    memset(&cmd_send_quque, 0, sizeof(cmd_send_quque));
    memset(&receive_pressure, 0, sizeof(receive_pressure));

    // 修改实时管理定时器频率为50Hz（每20ms执行一次）
    timer_rt_manager = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / 50),
        std::bind(&LZGripperNode::sendReceive_manager, this));

    // 添加新的压力值监控定时器（5Hz，每200ms执行一次）
    timer_pressure = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / 5),
        std::bind(&LZGripperNode::monitor_pressure, this));

    // 初始化压力值监控变量
    last_pressure_read_time = this->now();
    pressure_read_failures = 0;
    max_pressure_read_failures = 3;
  }

  /**
   * @brief 析构函数
   * 确保在节点销毁时断开与机械手的连接
   */
  ~LZGripperNode() {
    if (driver_) {
      driver_->disconnect();
    }
  }

private:
  /**
   * @brief 服务回调函数
   * 处理ROS2服务请求，执行相应的机械手控制命令
   */
  void service_handle(
      const std::shared_ptr<lz_gripper_ros2::srv::ServerMsg::Request> request,
      std::shared_ptr<lz_gripper_ros2::srv::ServerMsg::Response> response) {
    std::vector<float> pressure;
    
    switch (request->messageid) {
      case CMD_TYPE_SET_POSITION:  // 设置位置命令
        memset(&cmd_send_quque[cmd_send_queue_tail], 0, sizeof(send_cmd));
        cmd_send_quque[cmd_send_queue_tail].id = static_cast<uint8_t>(request->id);
        cmd_send_quque[cmd_send_queue_tail].delaytime = static_cast<uint32_t>(request->delay);
        cmd_send_quque[cmd_send_queue_tail].start = 0;
        cmd_send_quque[cmd_send_queue_tail].write_num = 6;
        
        // 设置所有手指的位置值
        for (int i = 0; i < 6; i++) {
          cmd_send_quque[cmd_send_queue_tail].val[i] = request->floatdata[0];
        }
        
        cmd_send_queue_tail = (cmd_send_queue_tail + 1) % QUEUE_NUM;
        
        // 设置响应
        response->okflag = 1;
        response->responseid = 2;
        response->pressure[0] = 111;
        response->pressure[1] = 112;
        response->pressure[2] = 113;
        response->pressure[3] = 114;
        response->pressure[4] = 115;
        break;

      case CMD_TYPE_GET_PRESSURE:  // 获取压力值命令
        if (!driver_->get_finger_pressure(pressure, request->id)) {
          RCLCPP_ERROR(this->get_logger(), "获取手指压力值失败");
          return;
        }
        
        // 设置响应
        response->okflag = 1;
        response->responseid = request->id;
        for (int i = 0; i < 5; i++) {
          response->pressure[i] = pressure.at(i);
        }
        python_delay = request->delay;
        break;

      default:
        break;
    }
  }

  /**
   * @brief 位置控制回调函数
   * 处理位置控制命令，将命令添加到队列中
   */
  void position_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    memset(&cmd_send_quque[cmd_send_queue_tail], 0, sizeof(send_cmd));
    cmd_send_quque[cmd_send_queue_tail].id = static_cast<uint8_t>(msg->data[0]);
    cmd_send_quque[cmd_send_queue_tail].delaytime = static_cast<uint32_t>(msg->data[1]);
    cmd_send_quque[cmd_send_queue_tail].start = 0;
    cmd_send_quque[cmd_send_queue_tail].write_num = 6;
    
    // 设置位置值
    for (int i = 0; i < 6; i++) {
      cmd_send_quque[cmd_send_queue_tail].val[i] = msg->data[i + 2];
    }
    
    cmd_send_queue_tail = (cmd_send_queue_tail + 1) % QUEUE_NUM;
  }

  /**
   * @brief 机械手命令回调函数
   * 处理开合等基本命令，将命令添加到队列中
   */
  void gripper_command_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
    memset(&cmd_send_quque[cmd_send_queue_tail], 0, sizeof(send_cmd));
    cmd_send_quque[cmd_send_queue_tail].id = static_cast<uint8_t>(msg->data[0]);
    cmd_send_quque[cmd_send_queue_tail].delaytime = static_cast<uint32_t>(msg->data[1]);
    cmd_send_quque[cmd_send_queue_tail].openorcloseflag = static_cast<uint8_t>(msg->data[2]);
    cmd_send_quque[cmd_send_queue_tail].start = 0;
    cmd_send_quque[cmd_send_queue_tail].write_num = 6;
    
    // 根据开合标志设置位置值
    if (cmd_send_quque[cmd_send_queue_tail].openorcloseflag == 0) {
      // 打开手指（位置值为0）
      for (int i = 0; i < 6; i++) {
        cmd_send_quque[cmd_send_queue_tail].val[i] = 0.f;
      }
    } else {
      // 关闭手指（位置值为1000）
      for (int i = 0; i < 6; i++) {
        cmd_send_quque[cmd_send_queue_tail].val[i] = 1000.f;
      }
    }
    
    cmd_send_queue_tail = (cmd_send_queue_tail + 1) % QUEUE_NUM;
  }

  /**
   * @brief 发送接收管理器
   * 处理命令队列和状态更新的主循环函数
   */
  void sendReceive_manager() {
    // 处理命令队列中的命令
    if (cmd_send_queue_head != cmd_send_queue_tail) {
      if (delay <= 0) {
        switch (cmd_send_quque[cmd_send_queue_head].type) {
          case CMD_TYPE_SET_POSITION:  // 设置位置
            if (!driver_->set_finger_positions(&cmd_send_quque[cmd_send_queue_head])) {
              RCLCPP_ERROR(this->get_logger(), "设置手指位置失败");
            } else {
              delay = cmd_send_quque[cmd_send_queue_head].delaytime;
              if (delay == 0) {
                cmd_send_queue_head = (cmd_send_queue_head + 1) % QUEUE_NUM;
              }
            }
            break;

          case CMD_TYPE_SET_VELOCITY:  // 设置速度
            if (!driver_->set_finger_velocity(&cmd_send_quque[cmd_send_queue_head])) {
              RCLCPP_ERROR(this->get_logger(), "设置手指速度失败");
            } else {
              delay = cmd_send_quque[cmd_send_queue_head].delaytime;
              if (delay == 0) {
                cmd_send_queue_head = (cmd_send_queue_head + 1) % QUEUE_NUM;
              }
            }
            break;

          default:
            break;
        }
      } else {
        delay--;
        if (delay <= 0) {
          cmd_send_queue_head = (cmd_send_queue_head + 1) % QUEUE_NUM;
        }
      }
    } else if (emergency_execution) {
      emergency_execution = 0;
      // 处理紧急情况
    }

    // 定期发布状态信息
    static int pressure_public_time = static_cast<int>((1000 / publish_rate));
    pressure_public_time--;
    if (pressure_public_time == 0) {
      pressure_public_time = static_cast<int>((1000 / publish_rate));
      
      // 获取压力值
      std::vector<float> positions;
      if (!driver_->get_finger_pressure(positions)) {
        RCLCPP_ERROR(this->get_logger(), "获取手指压力值失败");
        return;
      }

      // 发布关节状态
      auto joint_state = std::make_unique<sensor_msgs::msg::JointState>();
      joint_state->header.stamp = this->now();
      joint_state->name = {"finger1", "finger2", "finger3", "finger4", "finger5"};

      // 转换位置值为double类型
      std::vector<double> double_positions;
      double_positions.reserve(positions.size());
      for (float pos : positions) {
        double_positions.push_back(static_cast<double>(pos));
      }
      joint_state->position = double_positions;

      joint_state_pub_->publish(std::move(joint_state));

      // 发布机械手状态
      bool is_closed;
      if (driver_->get_gripper_state(is_closed)) {
        auto gripper_state = std::make_unique<std_msgs::msg::Bool>();
        gripper_state->data = is_closed;
        gripper_state_pub_->publish(std::move(gripper_state));
      }
    }
  }

  /**
   * @brief 命令插入回调函数
   * 统一处理各种控制命令的回调函数
   */
  void command_insert_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    python_delay--;
    while(callback_lock);  // 等待锁释放
    
    uint32_t callbacktype = static_cast<uint32_t>(msg->data[0]);
    
    switch(callbacktype) {
      case POSITION_SUB_CALLBACK:  // 位置控制
      {
        callback_lock = 1;  // 加锁
        
        memset(&cmd_send_quque[cmd_send_queue_tail], 0, sizeof(send_cmd));
        cmd_send_quque[cmd_send_queue_tail].id = static_cast<uint8_t>(msg->data[1]);
        cmd_send_quque[cmd_send_queue_tail].delaytime = static_cast<uint32_t>(msg->data[2]);
        cmd_send_quque[cmd_send_queue_tail].start = 0;
        cmd_send_quque[cmd_send_queue_tail].write_num = 6;
        
        // 设置位置值
        for (int i = 0; i < 6; i++) {
          cmd_send_quque[cmd_send_queue_tail].val[i] = msg->data[i + 3];
        }
        
        cmd_send_queue_tail = (cmd_send_queue_tail + 1) % QUEUE_NUM;
        callback_lock = 0;  // 释放锁
      }
      break;

      case GRIPPER_COMMAND_CALLBACK:  // 机械手命令
      {
        callback_lock = 1;
        
        memset(&cmd_send_quque[cmd_send_queue_tail], 0, sizeof(send_cmd));
        cmd_send_quque[cmd_send_queue_tail].id = static_cast<uint8_t>(msg->data[1]);
        cmd_send_quque[cmd_send_queue_tail].delaytime = static_cast<uint32_t>(msg->data[2]);
        cmd_send_quque[cmd_send_queue_tail].openorcloseflag = static_cast<uint8_t>(msg->data[3]);
        cmd_send_quque[cmd_send_queue_tail].start = 0;
        cmd_send_quque[cmd_send_queue_tail].write_num = 6;
        
        // 根据开合标志设置位置值
        if (cmd_send_quque[cmd_send_queue_tail].openorcloseflag == 0) {
          for (int i = 0; i < 6; i++) {
            cmd_send_quque[cmd_send_queue_tail].val[i] = 0.f;
          }
        } else {
          for (int i = 0; i < 6; i++) {
            cmd_send_quque[cmd_send_queue_tail].val[i] = 1000.f;
          }
        }
        
        cmd_send_queue_tail = (cmd_send_queue_tail + 1) % QUEUE_NUM;
        callback_lock = 0;
      }
      break;

      case VELOCITY_SUB_CALLBACK:  // 速度控制
      {
        callback_lock = 1;
        uint8_t flag = static_cast<uint8_t>(msg->data[2]);
        
        if (flag == 0) {  // 正常速度控制
          memset(&cmd_send_quque[cmd_send_queue_tail], 0, sizeof(send_cmd));
          cmd_send_quque[cmd_send_queue_tail].type = CMD_TYPE_SET_VELOCITY;
          cmd_send_quque[cmd_send_queue_tail].id = static_cast<uint8_t>(msg->data[1]);
          cmd_send_quque[cmd_send_queue_tail].emergencyflag = static_cast<uint8_t>(msg->data[2]);
          cmd_send_quque[cmd_send_queue_tail].delaytime = static_cast<int32_t>(msg->data[3]);
          cmd_send_quque[cmd_send_queue_tail].start = 6;
          cmd_send_quque[cmd_send_queue_tail].write_num = 6;
          
          // 设置速度值
          for (int i = 0; i < 6; i++) {
            cmd_send_quque[cmd_send_queue_tail].val[i] = msg->data[i + 4];
          }
          
          cmd_send_queue_tail = (cmd_send_queue_tail + 1) % QUEUE_NUM;
        } else {
          // 紧急情况处理
        }
        callback_lock = 0;
      }
      break;

      case EFFORT_SUB_CALLBACK:  // 力控制
      {
        callback_lock = 1;
        
        memset(&cmd_send_quque[cmd_send_queue_tail], 0, sizeof(send_cmd));
        cmd_send_quque[cmd_send_queue_tail].type = CMD_TYPE_SET_VELOCITY;
        cmd_send_quque[cmd_send_queue_tail].id = static_cast<uint8_t>(msg->data[1]);
        cmd_send_quque[cmd_send_queue_tail].emergencyflag = static_cast<uint8_t>(msg->data[2]);
        cmd_send_quque[cmd_send_queue_tail].delaytime = static_cast<int32_t>(msg->data[3]);
        cmd_send_quque[cmd_send_queue_tail].start = 12;
        cmd_send_quque[cmd_send_queue_tail].write_num = 6;
        
        // 设置力控制值
        for (int i = 0; i < 6; i++) {
          cmd_send_quque[cmd_send_queue_tail].val[i] = msg->data[i + 4];
        }
        
        cmd_send_queue_tail = (cmd_send_queue_tail + 1) % QUEUE_NUM;
        callback_lock = 0;
      }
      break;

      default:
        break;
    }
  }

  /**
   * @brief 发布关节状态
   * 获取并发布机械手的关节状态和压力值
   */
  void publish_joint_states() {
    std::vector<float> positions;
    if (!driver_->get_finger_pressure(positions)) {
      RCLCPP_ERROR(this->get_logger(), "获取手指压力值失败");
      return;
    }

    // 发布关节状态
    auto joint_state = std::make_unique<sensor_msgs::msg::JointState>();
    joint_state->header.stamp = this->now();
    joint_state->name = {"finger1", "finger2", "finger3", "finger4", "finger5"};

    // 转换位置值
    std::vector<double> double_positions;
    double_positions.reserve(positions.size());
    for (float pos : positions) {
      double_positions.push_back(static_cast<double>(pos));
    }
    joint_state->position = double_positions;

    joint_state_pub_->publish(std::move(joint_state));

    // 发布机械手状态
    bool is_closed;
    if (driver_->get_gripper_state(is_closed)) {
      auto gripper_state = std::make_unique<std_msgs::msg::Bool>();
      gripper_state->data = is_closed;
      gripper_state_pub_->publish(std::move(gripper_state));
    }
  }

  // 添加新的压力值监控函数
  void monitor_pressure() {
    // 如果距离上次成功读取时间太短，跳过这次读取
    auto current_time = this->now();
    if ((current_time - last_pressure_read_time).seconds() < 0.2) {
      return;
    }

    std::vector<float> pressure;
    // 尝试读取两个机械手的压力值
    for (uint8_t id = 1; id <= 2; id++) {
      if (driver_->get_finger_pressure(pressure, id)) {
        pressure_read_failures = 0;
        last_pressure_read_time = current_time;
        // 这里可以添加压力值处理逻辑
        return;
      }
    }

    // 记录失败次数
    pressure_read_failures++;
    if (pressure_read_failures >= max_pressure_read_failures) {
      RCLCPP_WARN(this->get_logger(), "连续%d次获取压力值失败", max_pressure_read_failures);
      pressure_read_failures = 0;  // 重置失败计数
    }
  }

  // 成员变量
  std::unique_ptr<lz_gripper_ros2::LZGripperDriver> driver_;  // 机械手驱动对象
  
  // ROS2发布者
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;    // 关节状态发布者
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gripper_state_pub_;          // 机械手状态发布者
  
  // ROS2订阅者
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr position_sub_;  // 位置控制订阅者
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr velocity_sub_;  // 速度控制订阅者
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr register_sub_;  // 寄存器控制订阅者
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr gripper_command_sub_;  // 机械手命令订阅者
  
  // ROS2服务和定时器
  rclcpp::Service<lz_gripper_ros2::srv::ServerMsg>::SharedPtr service_;           // 服务对象
  rclcpp::TimerBase::SharedPtr timer_;                                            // 定时器
  rclcpp::TimerBase::SharedPtr timer_rt_manager;                                  // 实时管理定时器
  rclcpp::TimerBase::SharedPtr timer_pressure;  // 压力值监控定时器
  rclcpp::Time last_pressure_read_time;         // 上次成功读取压力值的时间
  int pressure_read_failures;                    // 连续读取失败次数
  int max_pressure_read_failures;                // 最大允许连续失败次数

public:
  uint8_t callback_lock;      // 回调函数锁
  int32_t python_delay;       // Python接口延迟
  int32_t delay;              // 命令延迟
  uint8_t emergency_execution;  // 紧急执行标志
  receive_data receive_pressure;  // 压力值接收缓存
  int32_t cmd_send_queue_head;   // 命令队列头指针
  int32_t cmd_send_queue_tail;   // 命令队列尾指针
  double publish_rate;           // 发布频率
  send_cmd cmd_send_quque[QUEUE_NUM];  // 命令发送队列
};

/**
 * @brief 主函数
 * 初始化ROS2系统并运行节点
 */
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);                           // 初始化ROS2
  auto node = std::make_shared<LZGripperNode>();      // 创建节点
  rclcpp::spin(node);                                 // 运行节点
  rclcpp::shutdown();                                 // 关闭ROS2
  return 0;
}

#undef QUEUE_NUM  // 取消宏定义