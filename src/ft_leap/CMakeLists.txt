cmake_minimum_required(VERSION 3.5)
project(ft_leap)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate services in the 'srv' folder
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/LeapVelocity.srv"
  "srv/LeapPosition.srv"
  "srv/LeapEffort.srv"
  "srv/LeapPosVelEff.srv"
  DEPENDENCIES std_msgs
)

###################################
## ament specific configuration  ##
###################################
ament_package()

#############
## Install ##
#############
# Install the meshes directory
install(DIRECTORY
  assets/meshes
  DESTINATION share/${PROJECT_NAME}
)


# Install the Python utility module/directory
install(DIRECTORY
  scripts/leap_hand_utils
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/leapnode_FT.py
  scripts/leapnode_FT_topics.py
  scripts/leapnode_FT_topics_interp.py
  scripts/FT_client.py
  # scripts/leap_hand_utils
  
  # scripts/ros2_example.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# find_package(ament_cmake_gtest REQUIRED)
# ament_add_gtest(${PROJECT_NAME}-test test/test_leap_hand.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()
