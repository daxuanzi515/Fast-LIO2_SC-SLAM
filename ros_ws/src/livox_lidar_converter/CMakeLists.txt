cmake_minimum_required(VERSION 3.8)
project(livox_lidar_converter)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)

# Include headers
include_directories(include)

# Add executable
add_executable(lidar_converter src/lidar_converter.cpp)

# Link dependencies
ament_target_dependencies(lidar_converter rclcpp sensor_msgs livox_ros_driver2)

# Install
install(TARGETS
  lidar_converter
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})

ament_package()