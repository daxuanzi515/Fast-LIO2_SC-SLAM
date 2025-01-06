#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rclcpp/qos.hpp>
#include <memory>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

using std::placeholders::_1;

class ImuCsvToBag : public rclcpp::Node {
public:
  ImuCsvToBag()
  : Node("imu_csv_to_bag") {
    this->declare_parameter<std::string>("input_folder", "/home/cxx/Fast-LIO2_SC-SLAM/refs/examples/KAIST03_DATA");
    this->declare_parameter<std::string>("output_bag_prefix", "/home/cxx/Fast-LIO2_SC-SLAM/ros_ws/test_hub");
    input_folder_ = this->get_parameter("input_folder").as_string();
    output_bag_prefix_ = this->get_parameter("output_bag_prefix").as_string();
    imu_csv_path_ = input_folder_ + "/xsens_imu.csv";
    output_bag_path_ = output_bag_prefix_ + "/my_test.bag";

    RCLCPP_INFO(this->get_logger(), "Input CSV path: %s", imu_csv_path_.c_str());
    RCLCPP_INFO(this->get_logger(), "Output bag path: %s", output_bag_path_.c_str());



    // input_folder = ""
    // imu_csv_path_ = "/home/cxx/Fast-LIO2_SC-SLAM/refs/examples/KAIST03_DATA/xsens_imu.csv";
    // output_bag_path_ = "/home/cxx/Fast-LIO2_SC-SLAM/ros_ws/my_test.bag";

    const rosbag2_cpp::StorageOptions storage_options({output_bag_path_, "sqlite3"});
    const rosbag2_cpp::ConverterOptions converter_options(
      {rmw_get_serialization_format(),
       rmw_get_serialization_format()});
    writer_ = std::make_unique<rosbag2_cpp::writers::SequentialWriter>();

    writer_->open(storage_options, converter_options);

    writer_->create_topic(
      {"livox/imu",
       "sensor_msgs/msg/Imu",
       rmw_get_serialization_format(),
       ""});

    read_and_write_imu_data_to_bag();
  }

private:
  void read_and_write_imu_data_to_bag() {
    std::ifstream file(imu_csv_path_);
    if (!file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", imu_csv_path_.c_str());
        return;
    }

    std::string line;
    while (std::getline(file, line)) {
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

            // Create a Serialization object
            rclcpp::Serialization<sensor_msgs::msg::Imu> serializer;

            // Serialize the message into the SerializedMessage container
            rclcpp::SerializedMessage serialized_msg;
            serializer.serialize_message(&imu_msg, &serialized_msg);

            // Create a SerializedBagMessage
            auto bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
            bag_message->topic_name = "livox/imu";

            // Allocate a buffer for the serialized data and assign it to the bag message
            auto serialized_data = std::make_shared<rcutils_uint8_array_t>();
            serialized_data->buffer = serialized_msg.get_rcl_serialized_message().buffer;
            serialized_data->buffer_length = serialized_msg.get_rcl_serialized_message().buffer_length;

            // Set the serialized data in the bag message
            bag_message->serialized_data = serialized_data;

            // Set the time stamp
            if (rcutils_system_time_now(&bag_message->time_stamp) != RCUTILS_RET_OK) {
                RCLCPP_ERROR(this->get_logger(), "Error getting current time: %s",
                             rcutils_get_error_string().str);
            }

            // Write the message to the bag
            writer_->write(bag_message);
        }
    }
    RCLCPP_INFO(this->get_logger(), "Finished writing bag file: %s", output_bag_path_.c_str());
  }

  std::string input_folder_;
  std::string output_bag_prefix_;
  std::string imu_csv_path_;
  std::string output_bag_path_;
  std::unique_ptr<rosbag2_cpp::writers::SequentialWriter> writer_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImuCsvToBag>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
