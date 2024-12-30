#include "lidar_converter.hpp"

LidarConverter::LidarConverter() 
  : Node("lidar_converter")
{
  RCLCPP_INFO(this->get_logger(), "Starting LidarConverter node initialization.");
  pointcloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/os1_points",
    rclcpp::QoS(10),
    std::bind(&LidarConverter::callbackPointCloud, this, std::placeholders::_1)
  );

  // custommsg_publisher_ = this->create_publisher<livox_ros_driver2::msg::CustomMsg>(
  //   "/livox/lidar",
  //   rclcpp::QoS(10)
  // );
  custommsg_publisher_ = this->create_publisher<livox_ros_driver2::msg::CustomMsg>(
    "/livox/lidar",
    rclcpp::QoS(10).reliable().durability_volatile()
);

  RCLCPP_INFO(this->get_logger(), "Finished LidarConverter node initialization.");
}

void LidarConverter::callbackPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  if (msg->data.empty()) {
    RCLCPP_WARN(this->get_logger(), "Received empty PointCloud2 message.");
    return;
  }

  // Create an empty CustomMsg
  auto custom_msg = livox_ros_driver2::msg::CustomMsg();
  custom_msg.header = msg->header;
  custom_msg.timebase = msg->header.stamp.sec * 1e9 + msg->header.stamp.nanosec;
  custom_msg.lidar_id = 0;
  custom_msg.point_num = msg->width;

  const uint8_t* data_ptr = msg->data.data();
  std::vector<livox_ros_driver2::msg::CustomPoint> custom_points;
  custom_points.reserve(msg->width);

  for (unsigned int i = 0; i < msg->width; ++i) {
    const uint8_t* point_ptr = data_ptr + i * msg->point_step;

    float x, y, z, intensity;
    memcpy(&x, point_ptr + msg->fields[0].offset, sizeof(float));  // x
    memcpy(&y, point_ptr + msg->fields[1].offset, sizeof(float));  // y
    memcpy(&z, point_ptr + msg->fields[2].offset, sizeof(float));  // z
    memcpy(&intensity, point_ptr + msg->fields[3].offset, sizeof(float));  // intensity

    intensity = std::clamp(intensity, 0.0f, 255.0f);

    livox_ros_driver2::msg::CustomPoint custom_point;
    custom_point.x = x;
    custom_point.y = y;
    custom_point.z = z;
    custom_point.reflectivity = static_cast<uint8_t>(intensity + 0.5f);  // rounding
    custom_point.offset_time = 0;  // Optional: compute offset time if required
    custom_point.tag = 0;  // Default value
    custom_point.line = 0; // Default value

    custom_points.push_back(custom_point);
    }


  custom_msg.points = std::move(custom_points);

  // Publish the converted CustomMsg
  custommsg_publisher_->publish(custom_msg);
  RCLCPP_INFO(this->get_logger(), "Published %u points to /livox/lidar.", custom_msg.points.size());
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LidarConverter>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}