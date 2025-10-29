/*
 * Copyright (c) 2025 LZRobot. All rights reserved.
 * 灵掌机械手ROS2驱动程序实现文件
 * 
 * 该文件实现了与灵掌机械手的底层通信功能。
 * 主要实现：
 * 1. 串口通信的建立和管理
 * 2. Modbus RTU协议的数据包构建和解析
 * 3. 各种控制命令的具体实现
 */

#include "lz_gripper_ros2/lz_gripper_driver.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cstring>
#include <chrono>
#include <thread>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#define RETURN_STATUS_FALSE  false
#define __DEBUG 1
#define ERROR_REPORT RCLCPP_ERROR(rclcpp::get_logger("lz_gripper_driver"), "错误位置:%s %d",__FILE__,__LINE__);
namespace lz_gripper_ros2
{

// 构造函数：初始化串口文件描述符和连接状态
LZGripperDriver::LZGripperDriver() : serial_fd_(-1), is_connected_(false) {}

// 析构函数：确保断开连接
LZGripperDriver::~LZGripperDriver() {
  disconnect();
}

/**
 * @brief 连接到机械手
 * @param port 串口设备名
 * @param baud_rate 波特率
 * @return 连接是否成功
 */
bool LZGripperDriver::connect(const std::string& port, int baud_rate) {
  std::lock_guard<std::mutex> lock(mutex_);
  
  // 打开串口设备
  serial_fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (serial_fd_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("lz_gripper_driver"), 
                 "无法打开端口 %s", port.c_str());
    return false;
  }

  // 配置串口参数
  struct termios tty;
  if (tcgetattr(serial_fd_, &tty) != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("lz_gripper_driver"), 
                 "无法获取端口属性");
    close(serial_fd_);
    serial_fd_ = -1;
    return false;
  }

  // 设置波特率
  speed_t speed;
  switch (baud_rate) {
    case 9600:   speed = B9600;   break;
    case 19200:  speed = B19200;  break;
    case 38400:  speed = B38400;  break;
    case 57600:  speed = B57600;  break;
    case 115200: speed = B115200; break;
    default:
      RCLCPP_ERROR(rclcpp::get_logger("lz_gripper_driver"), 
                   "不支持的波特率: %d", baud_rate);
      close(serial_fd_);
      serial_fd_ = -1;
      return false;
  }

  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);

  // 配置为8N1模式，无流控制
  tty.c_cflag &= ~PARENB;        // 无校验位
  tty.c_cflag &= ~CSTOPB;        // 1位停止位
  tty.c_cflag &= ~CSIZE;         // 清除数据位设置
  tty.c_cflag |= CS8;            // 8位数据位
  tty.c_cflag &= ~CRTSCTS;       // 禁用硬件流控制
  tty.c_cflag |= CREAD | CLOCAL; // 启用接收器，忽略调制解调器控制线

  // 设置为原始输入模式
  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

  // 设置为原始输出模式
  tty.c_oflag &= ~OPOST;

  // 禁用软件流控制
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);

  // 应用设置
  if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("lz_gripper_driver"), 
                 "无法设置端口属性");
    close(serial_fd_);
    serial_fd_ = -1;
    return false;
  }

  is_connected_ = true;
  return true;
}

/**
 * @brief 断开与机械手的连接
 */
void LZGripperDriver::disconnect() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (serial_fd_ >= 0) {
    close(serial_fd_);
    serial_fd_ = -1;
  }
  is_connected_ = false;
}

/**
 * @brief 检查是否已连接
 */
bool LZGripperDriver::is_connected() const {
  return is_connected_;
}

/**
 * @brief 计算Modbus CRC校验码
 * @param data 数据
 * @param len 数据长度
 * @return CRC校验码
 */
uint16_t LZGripperDriver::modbus_crc(const uint8_t* data, int len) {
  uint16_t crc = 0xFFFF;
  for (int pos = 0; pos < len; pos++) {
    crc ^= (uint16_t)data[pos];
    for (int i = 8; i != 0; i--) {
      if ((crc & 0x0001) != 0) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

/**
 * @brief 发送Modbus请求
 * @param request 请求数据
 * @param len 数据长度
 * @return 发送是否成功
 */
bool LZGripperDriver::send_request(const uint8_t* request, int len) {
  if (!is_connected_) {
    #if __DEBUG==1
    ERROR_REPORT
    #endif
    return false;
  }
  
  ssize_t written = write(serial_fd_, request, len);
  if (written != len) {
    RCLCPP_ERROR(rclcpp::get_logger("lz_gripper_driver"), 
                 "无法写入所有字节");
                 #if __DEBUG==1
                 ERROR_REPORT
                 #endif   
    return false;
  }
  
  // 确保数据发送完成
  tcdrain(serial_fd_);
  return true;
}

/**
 * @brief 接收Modbus响应
 * @param response 响应数据缓冲区
 * @param max_len 最大接收长度
 * @param timeout_ms 超时时间（毫秒）
 * @return 接收到的字节数，失败返回-1
 */
int LZGripperDriver::receive_response(uint8_t* response, int max_len, int timeout_ms) {
  if (!is_connected_) {
    #if __DEBUG==1
                 ERROR_REPORT
                 #endif 
    return false;}
  
  auto start_time = std::chrono::steady_clock::now();
  int total_bytes = 0;
  
  while (total_bytes < max_len) {
    // Check timeout
    if (std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start_time).count() > timeout_ms) {
      break;
    }
    
    // Try to read available data
    int bytes_read = read(serial_fd_, response + total_bytes, max_len - total_bytes);

    if (bytes_read > 0) {
      total_bytes += bytes_read;

    } else if (bytes_read < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {

      RCLCPP_ERROR(rclcpp::get_logger("lz_gripper_driver"), 
                   "Read error: %s", strerror(errno));

      return false;
    }
    
    // Small delay to prevent busy waiting
    std::this_thread::sleep_for(std::chrono::milliseconds(1));

  }

  return total_bytes;
}

/**
 * @brief 写入单个寄存器
 * @param slave_addr 从机地址
 * @param reg_addr 寄存器地址
 * @param value 写入值
 * @return 操作是否成功
 */
bool LZGripperDriver::write_single_register(uint8_t slave_addr, uint16_t reg_addr, uint16_t value) {
  std::lock_guard<std::mutex> lock(mutex_);
  
  uint8_t request[8];
  request[0] = slave_addr;
  request[1] = 0x06;
  request[2] = (reg_addr >> 8) & 0xFF;
  request[3] = reg_addr & 0xFF;
  request[4] = (value >> 8) & 0xFF;
  request[5] = value & 0xFF;
  
  uint16_t crc = modbus_crc(request, 6);
  request[6] = crc & 0xFF;
  request[7] = (crc >> 8) & 0xFF;

  if (!send_request(request, 8)) return false;

  uint8_t response[8];
  if (!receive_response(response, 8, 100)) return false;

  // Verify response
  for (int i = 0; i < 6; i++) {
    if (response[i] != request[i]) return false;
  }

  crc = modbus_crc(response, 6);
  if (response[6] != (crc & 0xFF) || response[7] != ((crc >> 8) & 0xFF)) return false;

  return true;
}

/**
 * @brief 写入多个寄存器
 * @param slave_addr 从机地址
 * @param start_addr 起始地址
 * @param num_registers 寄存器数量
 * @param values 写入值数组
 * @return 操作是否成功
 */
bool LZGripperDriver::write_multiple_registers(uint8_t slave_addr, uint16_t start_addr, 
                                             uint16_t num_registers, const std::vector<uint16_t>& values) {
  std::lock_guard<std::mutex> lock(mutex_);
  
  #define RETURN_STATUS_FALSE  false
  std::vector<uint8_t> request(7 + num_registers * 2 + 2);
  request[0] = slave_addr;
  request[1] = 0x10;
  request[2] = (start_addr >> 8) & 0xFF;
  request[3] = start_addr & 0xFF;
  request[4] = (num_registers >> 8) & 0xFF;
  request[5] = num_registers & 0xFF;
  request[6] = num_registers * 2;

  for (size_t i = 1; i < values.size(); i++) {
    request[7 + 2*(i-1)] = (values[i] >> 8) & 0xFF;
    request[8 + 2*(i-1)] = values[i] & 0xFF;
  }

  uint16_t crc = modbus_crc(request.data(), request.size() - 2);
  request[request.size() - 2] = crc & 0xFF;
  request[request.size() - 1] = (crc >> 8) & 0xFF;
  #if __DEBUG==1
  RCLCPP_INFO(rclcpp::get_logger("lz_gripper_driver"), 
  "sent response: %d %d %d %d %d %d %d %d", request[0],request[1],
  request[7],request[8],request[9],request[10],request[11],request[12]);
#endif

  if (!send_request(request.data(), request.size())) return false;

  uint8_t response[8];
  if (!receive_response(response, 8, 100)) return false;
#if __DEBUG==1
RCLCPP_INFO(rclcpp::get_logger("lz_gripper_driver"), 
                   "get response: %d %d %d %d %d %d %d %d", response[0],response[1],
                   response[2],response[3],response[4],response[5],response[6],response[7]);
#endif
  // Verify response
  if (response[0] != slave_addr || response[1] != 0x10) return false;

  uint16_t resp_addr = (response[2] << 8) | response[3];
  uint16_t resp_num = (response[4] << 8) | response[5];
  if (resp_addr != start_addr || resp_num != num_registers) return false;

  crc = modbus_crc(response, 6);
  if (response[6] != (crc & 0xFF) || response[7] != ((crc >> 8) & 0xFF)) return false;

  return true;
}

/**
 * @brief 写入多个寄存器（命令方式）
 * @param cmd 命令结构体
 * @return 操作是否成功
 */
bool LZGripperDriver::write_multiple_registers(send_cmd* cmd) {
  std::lock_guard<std::mutex> lock(mutex_);
  
  #define RETURN_STATUS_FALSE  false
  //std::vector<uint8_t> request(7 + num_registers * 2 + 2);
  
  cmd->cmd[0] = cmd->id;
  cmd->cmd[1] = 0x10;
  cmd->cmd[2] = (cmd->start >> 8) & 0xFF;
  cmd->cmd[3] = cmd->start & 0xFF;
  cmd->cmd[4] = (cmd->write_num >> 8) & 0xFF;
  cmd->cmd[5] = cmd->write_num & 0xFF;
  cmd->cmd[6] = cmd->write_num * 2;
  cmd->size=7 + cmd->write_num * 2+2;
  for (size_t i = 0; i < cmd->write_num; i++) {
    cmd->cmd[7 + 2*(i)] = (cmd->pos[i] >> 8) & 0xFF;
    cmd->cmd[8 + 2*(i)] = cmd->pos[i] & 0xFF;
  }

  uint16_t crc = modbus_crc(cmd->cmd, cmd->size-2);
  cmd->cmd[cmd->size-2] = crc & 0xFF;
  cmd->cmd[cmd->size-1] = (crc >> 8) & 0xFF;
 

  if (!send_request(cmd->cmd, cmd->size)) return false;

  if (!receive_response(cmd->response, 8, 100)) return false;

  // Verify response
  if (cmd->response[0] != cmd->id || cmd->response[1] != 0x10) return false;

  uint16_t resp_addr = (cmd->response[2] << 8) | cmd->response[3];
  uint16_t resp_num = (cmd->response[4] << 8) | cmd->response[5];
  if (resp_addr != cmd->start || resp_num != cmd->write_num) return false;

  crc = modbus_crc(cmd->response, 6);
  if (cmd->response[6] != (crc & 0xFF) || cmd->response[7] != ((crc >> 8) & 0xFF)) return false;

  return true;
}

/**
 * @brief 读取保持寄存器
 * @param slave_addr 从机地址
 * @param start_addr 起始地址
 * @param num_registers 寄存器数量
 * @param values 读取值数组
 * @return 操作是否成功
 */
bool LZGripperDriver::read_holding_registers(uint8_t slave_addr, uint16_t start_addr, 
                                           uint16_t num_registers, std::vector<uint16_t>& values) {
  std::lock_guard<std::mutex> lock(mutex_);
  
  uint8_t request[8];
  request[0] = slave_addr;
  request[1] = 0x03;
  request[2] = (start_addr >> 8) & 0xFF;
  request[3] = start_addr & 0xFF;
  request[4] = (num_registers >> 8) & 0xFF;
  request[5] = num_registers & 0xFF;
  
  uint16_t crc = modbus_crc(request, 6);
  request[6] = crc & 0xFF;
  request[7] = (crc >> 8) & 0xFF;

  if (send_request(request, 8)!=true) {
    #if __DEBUG==1
    ERROR_REPORT
    #endif
    return RETURN_STATUS_FALSE;}

  uint8_t response[256];
  int response_len =receive_response(response, sizeof(response), 100);
  if (response_len < 5) {
    #if __DEBUG==1
    RCLCPP_ERROR(rclcpp::get_logger("lz_gripper_driver"), "response_len:%d",response_len);
    ERROR_REPORT
    #endif
    return RETURN_STATUS_FALSE;
  }

  // Verify response
  crc = modbus_crc(response, response_len - 2);
  if (response[response_len - 2] != (crc & 0xFF) || response[response_len - 1] != ((crc >> 8) & 0xFF)) {
    #if __DEBUG==1
    ERROR_REPORT
    #endif
    return RETURN_STATUS_FALSE;
}

  uint8_t byte_count = response[2];
  if (byte_count != num_registers * 2) {
    #if __DEBUG==1
    ERROR_REPORT
    #endif
    return RETURN_STATUS_FALSE;}


  values.resize(num_registers);
  for (size_t i = 0; i < num_registers; i++) {
    values[i] = (response[3 + 2*i] << 8) | response[4 + 2*i];
  }
  #if __DEBUG==1
 //RCLCPP_INFO(rclcpp::get_logger("lz_gripper_driver"), "got values:%d %d %d %d %d",values.at(0),
 //values.at(1),values.at(2),values.at(3),values.at(4));
  #endif
  return true;
}

/**
 * @brief 设置手指位置
 * @param positions 位置数组
 * @return 操作是否成功
 */
bool LZGripperDriver::set_finger_positions(const std::vector<float>& positions) {
  if (positions.size() != NUM_SET_POSITON_PARA) {
    RCLCPP_ERROR(rclcpp::get_logger("lz_gripper_driver"), 
                 "位置数量无效。期望 %d, 实际 %zu", 
                 NUM_SET_POSITON_PARA, positions.size());
    return false;
  }
  if(positions.at(0)!=2&&positions.at(0)!=1){
    RCLCPP_ERROR(rclcpp::get_logger("lz_gripper_driver"), 
                 "机械手ID错误。期望 1 或 2, 实际 %f", 
                 positions.at(0));
    return false;
  }
  for(int i=1;i<NUM_SET_POSITON_PARA;i++)
  if(positions.at(i)<MIN_POSITION||positions.at(i)>MAX_POSITION){
    RCLCPP_ERROR(rclcpp::get_logger("lz_gripper_driver"), 
    "手指位置错误。位置 %d 期望 0~1000, 实际 %0.1f", 
    i,positions.at(i));
  }
  
  // 将位置值转换为uint16_t类型
  std::vector<uint16_t> values;
  values.reserve(NUM_SET_POSITON_PARA-1);
  
  for (float pos : positions) {
    // 将位置限制在MIN_POSITION和MAX_POSITION之间
    pos = std::max(MIN_POSITION, std::min(pos, MAX_POSITION));
    values.push_back(static_cast<uint16_t>(pos));
  }

  return write_multiple_registers(values.at(0), 0, NUM_FINGERS+1, values);
}

/**
 * @brief 设置手指位置（命令方式）
 * @param cmd 命令结构体
 * @return 操作是否成功
 */
bool LZGripperDriver::set_finger_positions(send_cmd* cmd) {
  if(cmd->id!=2&&cmd->id!=1){
    RCLCPP_ERROR(rclcpp::get_logger("lz_gripper_driver"), 
                 "机械手ID错误。期望 1 或 2, 实际 %d", 
                 cmd->id);
    return false;
  }
  for(int i=0;i<NUM_SET_POSITON_PARA-1;i++)
  if(cmd->val[i]<MIN_POSITION||cmd->val[i]>MAX_POSITION){
    RCLCPP_ERROR(rclcpp::get_logger("lz_gripper_driver"), 
    "手指位置错误。位置 %d 期望 0~1000, 实际 %0.1f", 
    i,cmd->val[i]);
  }

  // 将位置值转换并限制在有效范围内
  for (int i=0;i<6;i++) {
    cmd->pos[i]=static_cast<uint16_t>(std::max(MIN_POSITION, std::min(cmd->val[i], MAX_POSITION)));
  }

  return write_multiple_registers(cmd);
}

/**
 * @brief 获取手指压力值
 * @param positions 存储压力值的数组
 * @return 操作是否成功
 */
bool LZGripperDriver::get_finger_pressure(std::vector<float>& positions) {
  std::vector<uint16_t> values;
  static uint8_t register_id=1;
  static uint8_t count=0;
  
  // 尝试读取压力值
  if (false==read_holding_registers(register_id, 18, NUM_FINGERS, values)) {
    count++;
    if((count%2)==0)
      register_id=2;
    else
      register_id=1;
    return false;
  }

  // 转换为浮点数
  positions.resize(NUM_FINGERS);
  for (size_t i = 0; i < NUM_FINGERS; i++) {
    positions[i] = static_cast<float>(values[i]);
  }

  return true;
}

/**
 * @brief 获取指定ID机械手的压力值
 * @param positions 存储压力值的数组
 * @param id 机械手ID
 * @return 操作是否成功
 */
bool LZGripperDriver::get_finger_pressure(std::vector<float>& positions,uint8_t id) {
  std::vector<uint16_t> values;
  if (false==read_holding_registers(id, 18, NUM_FINGERS, values)) {
    return false;
  }

  positions.resize(NUM_FINGERS);
  for (size_t i = 0; i < NUM_FINGERS; i++) {
    positions[i] = static_cast<float>(values[i]);
  }

  return true;
}

/**
 * @brief 设置手指速度
 * @param cmd 速度命令结构体
 * @return 操作是否成功
 */
bool LZGripperDriver::set_finger_velocity(send_cmd* cmd) {
  if(cmd->id!=2&&cmd->id!=1){
    RCLCPP_ERROR(rclcpp::get_logger("lz_gripper_driver"), 
                 "机械手ID错误。期望 1 或 2, 实际 %d", 
                 cmd->id);
    return false;
  }
  for(int i=0;i<6;i++)
  if(cmd->val[i]<MIN_POSITION||cmd->val[i]>MAX_POSITION){
    RCLCPP_ERROR(rclcpp::get_logger("lz_gripper_driver"), 
    "速度值错误。位置 %d 期望 0~1000, 实际 %0.1f", 
    i,cmd->val[i]);
  }

  // 限制速度值在有效范围内
  for (int i=0;i<6;i++) {
    cmd->pos[i]=static_cast<uint16_t>(std::max(MIN_POSITION, std::min(cmd->val[i], MAX_POSITION)));
  }
  return write_multiple_registers(cmd);
}

/**
 * @brief 设置机械手状态
 * @param param 参数数组（包含ID和开合状态）
 * @return 操作是否成功
 */
bool LZGripperDriver::set_gripper_state(const std::vector<int32_t>& param) {
  std::vector<float> positions(NUM_SET_POSITON_PARA, param[1] ? MAX_POSITION : MIN_POSITION);
  positions[0]=param[0];
  return set_finger_positions(positions);
}

/**
 * @brief 获取机械手状态
 * @param is_closed 存储机械手是否闭合的状态
 * @return 操作是否成功
 */
bool LZGripperDriver::get_gripper_state(bool& is_closed) {
  std::vector<float> positions;
  if (!get_finger_pressure(positions)) {
    return false;
  }

  // 如果所有手指都接近最大位置，则认为机械手处于闭合状态
  is_closed = true;
  for (float pos : positions) {
    if (pos < MAX_POSITION * 0.9f) {  // 90%阈值
      is_closed = false;
      break;
    }
  }

  return true;
}

} // namespace lz_gripper_ros2 