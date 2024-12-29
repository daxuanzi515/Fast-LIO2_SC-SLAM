cmake_minimum_required(VERSION 3.5)
project(aloam_velodyne)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
# find_package(rosbag REQUIRED)
find_package(std_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)

set(GTSAM_DIR "/usr/local/lib/cmake/GTSAM")
find_package(GTSAM 4.2.0 EXACT REQUIRED)
message(STATUS "GTSAM version: ${GTSAM_VERSION}")

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
)

# catkin_package(
#   CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
#   DEPENDS EIGEN3 PCL 
#   INCLUDE_DIRS include
# )

ament_package()

# add_executable(ascanRegistration src/scanRegistration.cpp)
# ament_target_dependencies(ascanRegistration rclcpp rclpy std_msgs geometry_msgs nav_msgs sensor_msgs)
# target_link_libraries(ascanRegistration ${PCL_LIBRARIES})

# add_executable(alaserOdometry src/laserOdometry.cpp)
# ament_target_dependencies(alaserOdometry rclcpp rclpy std_msgs geometry_msgs nav_msgs sensor_msgs)
# target_link_libraries(alaserOdometry ${PCL_LIBRARIES} ${CERES_LIBRARIES})

# add_executable(alaserMapping src/laserMapping.cpp)
# ament_target_dependencies(alaserMapping rclcpp rclpy std_msgs geometry_msgs nav_msgs sensor_msgs tf2 tf2_ros)
# target_link_libraries(alaserMapping ${PCL_LIBRARIES} ${CERES_LIBRARIES})

add_executable(alaserPGO 
  src/laserPosegraphOptimization.cpp
  include/scancontext/Scancontext.cpp
)
ament_target_dependencies(alaserPGO rclcpp rclpy std_msgs geometry_msgs nav_msgs sensor_msgs tf2 tf2_ros)
target_compile_options(alaserPGO 
  PRIVATE ${OpenMP_CXX_FLAGS}
)
target_link_libraries(alaserPGO 
  ${PCL_LIBRARIES} ${CERES_LIBRARIES}
  ${OpenMP_CXX_FLAGS} ${GTSAM_LIBRARIES}
  gtsam
)
install(DIRECTORY
  ${CMAKE_SOURCE_DIR}/launch
  DESTINATION share/aloam_velodyne/launch)
 
install(TARGETS alaserPGO
  DESTINATION lib/aloam_velodyne)
  
# add_executable(kittiHelper src/kittiHelper.cpp)
# ament_target_dependencies(kittiHelper rclcpp rclpy std_msgs geometry_msgs nav_msgs sensor_msgs)
# target_link_libraries(kittiHelper ${PCL_LIBRARIES} ${OpenCV_LIBS})

