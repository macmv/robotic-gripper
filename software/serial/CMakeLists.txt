cmake_minimum_required(VERSION 3.16)
project(robot_gripper)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(robot_gripper_node src/main.cpp)
ament_target_dependencies(robot_gripper_node rclcpp std_msgs std_srvs)

install(TARGETS robot_gripper_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
