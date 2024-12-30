#ifndef LIDAR_CONVERTER_HPP
#define LIDAR_CONVERTER_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "livox_ros_driver2/msg/custom_msg.hpp"

class LidarConverter : public rclcpp::Node {
public:
  LidarConverter();
  
private:
  void callbackPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscription_;
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr custommsg_publisher_;
};

#endif // LIDAR_CONVERTER_HPP