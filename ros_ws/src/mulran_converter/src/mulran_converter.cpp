#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>
#include <rosbag2_cpp/writer.hpp>
#include <Eigen/Dense>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <boost/filesystem.hpp>
#include <fstream>
#include <vector>
#include <string>
#include <iostream>

namespace bfs = boost::filesystem;

struct PointXYZIRT {
  PCL_ADD_POINT4D;
  float intensity;
  uint32_t t;
  int ring;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT,
                                   (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint32_t, t, t)(int, ring, ring))

class Mulran2Bag : public rclcpp::Node {
public:
  Mulran2Bag() : Node("mulran2bag") {}

  void convertDataset(const std::string &mulran_folder, const std::string &bag_name) {
    auto writer = std::make_shared<rosbag2_cpp::Writer>();
    writer->open(bag_name);

    // LIDAR Data
    writeLidarData(writer, mulran_folder);

    // IMU Data
    writeIMUData(writer, mulran_folder);

    // GPS Data
    writeGPSData(writer, mulran_folder);

    RCLCPP_INFO(get_logger(), "Conversion completed for bag: %s", bag_name.c_str());
  }

private:
  void writeLidarData(std::shared_ptr<rosbag2_cpp::Writer> writer, const std::string &mulran_folder) {
    std::string custom_topic = "livox/lidar";
    bfs::path ouster_stamps_csv = bfs::path(mulran_folder) / "sensor_data" / "ouster_front_stamp.csv";
    std::vector<uint64_t> stamps;

    std::ifstream fin(ouster_stamps_csv.string());
    if (fin.is_open()) {
      uint64_t stamp;
      while (fin >> stamp) {
        stamps.push_back(stamp);
      }
      fin.close();
    }

    for (const auto &stamp : stamps) {
      std::stringstream ss;
      ss << stamp << ".bin";
      bfs::path ouster_data_path = bfs::path(mulran_folder) / "sensor_data" / "Ouster" / ss.str();
      if (!bfs::exists(ouster_data_path)) {
        RCLCPP_WARN(get_logger(), "Cloud %s does not exist! Skipping.", ouster_data_path.string().c_str());
        continue;
      }

      pcl::PointCloud<PointXYZIRT> cloud;
      cloud.clear();
      sensor_msgs::msg::PointCloud2 publish_cloud;

      std::ifstream file(ouster_data_path.string(), std::ios::in | std::ios::binary);
      if (!file.is_open()) {
        RCLCPP_ERROR(get_logger(), "Could not open file %s", ouster_data_path.string().c_str());
        continue;
      }

      while (!file.eof()) {
        PointXYZIRT point;
        file.read(reinterpret_cast<char *>(&point.x), sizeof(float));
        file.read(reinterpret_cast<char *>(&point.y), sizeof(float));
        file.read(reinterpret_cast<char *>(&point.z), sizeof(float));
        file.read(reinterpret_cast<char *>(&point.intensity), sizeof(float));
        point.ring = 1;  // Example value
        cloud.points.push_back(point);
      }
      file.close();

      pcl::toROSMsg(cloud, publish_cloud);
      publish_cloud.header.stamp = rclcpp::Time(stamp);
      publish_cloud.header.frame_id = "os1_lidar";

      writeCustomMsg(writer, custom_topic, publish_cloud);
    }
  }

  void writeCustomMsg(std::shared_ptr<rosbag2_cpp::Writer> writer, const std::string &topic, const sensor_msgs::msg::PointCloud2 &cloud_msg) {
    if (cloud_msg.data.empty()) {
      RCLCPP_WARN(this->get_logger(), "Received empty PointCloud2 message.");
      return;
    }

    auto custom_msg = livox_ros_driver2::msg::CustomMsg();
    custom_msg.header = cloud_msg.header;
    custom_msg.timebase = cloud_msg.header.stamp.sec * 1e9 + cloud_msg.header.stamp.nanosec;
    custom_msg.lidar_id = 0;
    custom_msg.point_num = cloud_msg.width;

    const uint8_t *data_ptr = cloud_msg.data.data();
    std::vector<livox_ros_driver2::msg::CustomPoint> custom_points;
    custom_points.reserve(cloud_msg.width);

    for (unsigned int i = 0; i < cloud_msg.width; ++i) {
      const uint8_t *point_ptr = data_ptr + i * cloud_msg.point_step;

      float x, y, z, intensity;
      memcpy(&x, point_ptr + cloud_msg.fields[0].offset, sizeof(float));  // x
      memcpy(&y, point_ptr + cloud_msg.fields[1].offset, sizeof(float));  // y
      memcpy(&z, point_ptr + cloud_msg.fields[2].offset, sizeof(float));  // z
      memcpy(&intensity, point_ptr + cloud_msg.fields[3].offset, sizeof(float));  // intensity

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

    // Write to bag
    writer->write(
        std::make_shared<rosbag2_storage::SerializedBagMessage>(rclcpp::SerializedMessage(custom_msg)),
        topic, "livox_ros_driver2/msg/CustomMsg", cloud_msg.header.stamp.sec * 1e9 + cloud_msg.header.stamp.nanosec);

    RCLCPP_INFO(this->get_logger(), "Wrote %u points to topic: %s", custom_msg.points.size(), topic.c_str());
  }

  void writeIMUData(std::shared_ptr<rosbag2_cpp::Writer> writer, const std::string &mulran_folder) {
    std::string imu_topic = "imu/data_raw";
    bfs::path imu_csv_path = bfs::path(mulran_folder) / "sensor_data" / "xsens_imu.csv";

    std::ifstream fin(imu_csv_path.string());
    if (!fin.is_open()) {
      RCLCPP_ERROR(get_logger(), "Could not open IMU file: %s", imu_csv_path.string().c_str());
      return;
    }

    std::string line;
    while (std::getline(fin, line)) {
      std::istringstream ss(line);
      uint64_t stamp;
      double q_x, q_y, q_z, q_w, g_x, g_y, g_z, a_x, a_y, a_z;
      if (ss >> stamp >> q_x >> q_y >> q_z >> q_w >> g_x >> g_y >> g_z >> a_x >> a_y >> a_z) {
        sensor_msgs::msg::Imu imu_msg;
        imu_msg.header.stamp = rclcpp::Time(stamp);
        imu_msg.header.frame_id = "imu";
        imu_msg.orientation.x = q_x;
        imu_msg.orientation.y = q_y;
        imu_msg.orientation.z = q_z;
        imu_msg.orientation.w = q_w;
        imu_msg.angular_velocity.x = g_x;
        imu_msg.angular_velocity.y = g_y;
        imu_msg.angular_velocity.z = g_z;
        imu_msg.linear_acceleration.x = a_x;
        imu_msg.linear_acceleration.y = a_y;
        imu_msg.linear_acceleration.z = a_z;

        writer->write(
            std::make_shared<rosbag2_storage::SerializedBagMessage>(rclcpp::SerializedMessage(imu_msg)),
            imu_topic, "sensor_msgs/msg/Imu", rclcpp::Time(stamp).nanoseconds());
      }
    }
  }

  void writeGPSData(std::shared_ptr<rosbag2_cpp::Writer> writer, const std::string &mulran_folder) {
    std::string gps_topic = "gps/fix";
    bfs::path gps_csv_path = bfs::path(mulran_folder) / "sensor_data" / "gps.csv";

    std::ifstream fin(gps_csv_path.string());
    if (!fin.is_open()) {
      RCLCPP_ERROR(get_logger(), "Could not open GPS file: %s", gps_csv_path.string().c_str());
      return;
    }

    std::string line;
    while (std::getline(fin, line)) {
      std::istringstream ss(line);
      uint64_t stamp;
      double latitude, longitude, altitude;
      if (ss >> stamp >> latitude >> longitude >> altitude) {
        sensor_msgs::msg::NavSatFix gps_msg;
        gps_msg.header.stamp = rclcpp::Time(stamp);
        gps_msg.header.frame_id = "gps";
        gps_msg.latitude = latitude;
        gps_msg.longitude = longitude;
        gps_msg.altitude = altitude;

        writer->write(
            std::make_shared<rosbag2_storage::SerializedBagMessage>(rclcpp::SerializedMessage(gps_msg)),
            gps_topic, "sensor_msgs/msg/NavSatFix", rclcpp::Time(stamp).nanoseconds());
      }
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  if (argc < 3) {
    std::cerr << "Usage: ros2 run mulran_converter mulran2bag <mulran_folder> <bag_name>" << std::endl;
    return 1;
  }
  auto node = std::make_shared<Mulran2Bag>();
  std::string mulran_folder = argv[1];
  std::string bag_name = argv[2];
  node->convertDataset(mulran_folder, bag_name);
  rclcpp::shutdown();
  return 0;
}
// ros2 run mulran_converter mulran_converter <mulran_folder> <output_bag_name>