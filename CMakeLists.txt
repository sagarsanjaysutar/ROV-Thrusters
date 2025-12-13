cmake_minimum_required(VERSION 3.8)
project(rov_project)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)


add_executable(thruster_control src/ThrusterControl.cpp)
ament_target_dependencies(thruster_control rclcpp geometry_msgs mavros_msgs)

install(TARGETS thruster_control DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
