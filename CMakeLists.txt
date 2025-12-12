cmake_minimum_required(VERSION 3.8)
project(rov_project)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)


add_executable(velocity_publisher src/VelocityPublisher.cpp)
ament_target_dependencies(velocity_publisher rclcpp std_msgs geometry_msgs)

add_executable(thruster_publisher src/ThrusterPublisher.cpp)
ament_target_dependencies(thruster_publisher rclcpp std_msgs geometry_msgs)


add_executable(mavros_bridge src/MavrosBridge.cpp)
ament_target_dependencies(mavros_bridge rclcpp std_msgs mavros_msgs)

install(TARGETS velocity_publisher thruster_publisher mavros_bridge DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
