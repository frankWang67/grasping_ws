cmake_minimum_required(VERSION 3.8)
project(lz_gripper_ros2)

# 编译器选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 生成接口
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ServerMsg.srv"
  "action/HandCommand.action"
  DEPENDENCIES std_msgs
)

# C++部分
add_library(lz_gripper_driver
  src/lz_gripper_driver.cpp
)

target_include_directories(lz_gripper_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(lz_gripper_driver
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
)

add_executable(lz_gripper_node src/lz_gripper_node.cpp)
target_link_libraries(lz_gripper_node lz_gripper_driver)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(lz_gripper_node "${cpp_typesupport_target}")

# 安装
install(TARGETS
    lz_gripper_driver
    lz_gripper_node
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Python节点安装
install(PROGRAMS
  nodes/hand_controller_node.py
  nodes/test_hand_client.py
  DESTINATION lib/${PROJECT_NAME}
)

# 导出依赖
ament_export_dependencies(rosidl_default_runtime)
ament_package()
