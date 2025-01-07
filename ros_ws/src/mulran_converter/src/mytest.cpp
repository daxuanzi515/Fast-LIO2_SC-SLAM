#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp> 
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rclcpp/qos.hpp>
#include "livox_ros_driver2/msg/custom_msg.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <memory>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <queue>
#include <map>
#include <cstdio>
#include "boost/lexical_cast.hpp" 
#include <dirent.h>

using std::placeholders::_1;

template <typename T>
struct DataThread{
  std::queue<T> data_queue_;
  DataThread() {}
  void push(T data){
    data_queue_.push(data);
  }
  T pop(){
    T result;
    result = data_queue_.front();
    data_queue_.pop();
    return result;
  }
};

struct PointXYZIRT {
  PCL_ADD_POINT4D;
  float intensity;
  uint32_t t;
  int ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRT,
                                   (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
                                   (uint32_t, t, t) (int, ring, ring)
                                   )

class CsvToBag : public rclcpp::Node {
public:
    CsvToBag() : Node("csv_to_bag") {
        this->declare_parameter<std::string>("input_folder", "/home/cxx/Fast-LIO2_SC-SLAM/refs/examples/KAIST03_DATA");
        this->declare_parameter<std::string>("output_bag_prefix", "/home/cxx/Fast-LIO2_SC-SLAM/ros_ws/test_hub");
        
        input_folder_ = this->get_parameter("input_folder").as_string();
        output_bag_prefix_ = this->get_parameter("output_bag_prefix").as_string();
        imu_csv_path_ = input_folder_ + "/xsens_imu.csv";
        gps_csv_path_ = input_folder_ + "/gps.csv";
        data_stamp_csv_path_ = input_folder_ + "/data_stamp.csv";
        ouster_folder_path_ = input_folder_ + "/Ouster";
        output_bag_path_ = output_bag_prefix_ + "/DCC02.bag";

        RCLCPP_INFO(this->get_logger(), "Input CSV paths: %s, %s", imu_csv_path_.c_str(), gps_csv_path_.c_str());
        RCLCPP_INFO(this->get_logger(), "Output bag path: %s", output_bag_path_.c_str());

        const rosbag2_cpp::StorageOptions storage_options({output_bag_path_, "sqlite3"});
        const rosbag2_cpp::ConverterOptions converter_options(
            {rmw_get_serialization_format(), rmw_get_serialization_format()});
        writer_ = std::make_unique<rosbag2_cpp::writers::SequentialWriter>();

        writer_->open(storage_options, converter_options);

        writer_->create_topic(
            {"/livox/imu", "sensor_msgs/msg/Imu", rmw_get_serialization_format(), ""});

        writer_->create_topic(
            {"/gps/fix", "sensor_msgs/msg/NavSatFix", rmw_get_serialization_format(), ""});

        writer_->create_topic(
            {"/raw_points", "sensor_msgs/msg/PointCloud2", rmw_get_serialization_format(), ""});
        
        writer_->create_topic(
            {"/livox/lidar", "livox_ros_driver2/msg/CustomMsg", rmw_get_serialization_format(), ""});

        ReadDataFromFile();
        read_and_write_imu_data_to_bag();
        read_and_write_gps_data_to_bag();
        read_and_write_ouster_data_to_bag();
    }

    ~CsvToBag() {}

private:
void ReadDataFromFile() {
    // 读取关键帧时间戳
    std::ifstream f(data_stamp_csv_path_);
    if (!f.good()) {
        RCLCPP_ERROR(this->get_logger(), "Data stamp file does not exist.");
        return;
    }

    FILE *fp;
    int64_t stamp;
    char data_name[50];
    data_stamp_.clear();

    // 读取关键帧时间戳和文件名（数据标识符）
    fp = fopen(data_stamp_csv_path_.c_str(), "r");
    while (fscanf(fp, "%ld,%s\n", &stamp, data_name) == 2) {
        data_stamp_.insert({stamp, data_name});
    }
    fclose(fp);

    RCLCPP_INFO(this->get_logger(), "Stamp data are loaded.");


    for(auto iter = data_stamp_.begin() ; iter != data_stamp_.end() ; iter ++) {
    auto stamp = iter->first;
    if(iter->second.compare("imu") == 0) {
      imu_thread_.push(stamp);
    } else if(iter->second.compare("gps") == 0) {
      gps_thread_.push(stamp);
    } else if(iter->second.compare("ouster") == 0) {
      ouster_thread_.push(stamp);
    }
  }
}

void read_and_write_imu_data_to_bag() {
    FILE *fp = fopen(imu_csv_path_.c_str(), "r");
    if (!fp) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open IMU data file: %s", imu_csv_path_.c_str());
        return;
    }

    int64_t stamp;
    double q_x, q_y, q_z, q_w, x, y, z, g_x, g_y, g_z, a_x, a_y, a_z, m_x, m_y, m_z;
    sensor_msgs::msg::Imu imu_data;
    imu_data_.clear();
    while(1) {
      int length = fscanf(fp,"%ld,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n", \
                          &stamp,&q_x,&q_y,&q_z,&q_w,&x,&y,&z,&g_x,&g_y,&g_z,&a_x,&a_y,&a_z,&m_x,&m_y,&m_z);
      if(length != 8 && length != 17)
        break;
      if(length == 8) {
        imu_data.header.stamp = rclcpp::Time(stamp);
        imu_data.header.frame_id = "imu";
        imu_data.orientation.x = q_x;
        imu_data.orientation.y = q_y;
        imu_data.orientation.z = q_z;
        imu_data.orientation.w = q_w;

        imu_data_[stamp] = imu_data;
        imu_data_version_ = 1;
      } else if(length == 17) {
        imu_data.header.stamp = rclcpp::Time(stamp);
        imu_data.header.frame_id = "imu";
        imu_data.orientation.x = q_x;
        imu_data.orientation.y = q_y;
        imu_data.orientation.z = q_z;
        imu_data.orientation.w = q_w;
        imu_data.angular_velocity.x = g_x;
        imu_data.angular_velocity.y = g_y;
        imu_data.angular_velocity.z = g_z;
        imu_data.linear_acceleration.x = a_x;
        imu_data.linear_acceleration.y = a_y;
        imu_data.linear_acceleration.z = a_z;

        imu_data.orientation_covariance[0] = 3;
        imu_data.orientation_covariance[4] = 3;
        imu_data.orientation_covariance[8] = 3;
        imu_data.angular_velocity_covariance[0] = 3;
        imu_data.angular_velocity_covariance[4] = 3;
        imu_data.angular_velocity_covariance[8] = 3;
        imu_data.linear_acceleration_covariance[0] = 3;
        imu_data.linear_acceleration_covariance[4] = 3;
        imu_data.linear_acceleration_covariance[8] = 3;
        imu_data_[stamp] = imu_data;

        imu_data_version_ = 2;
      }
    }
    fclose(fp);
    while(!imu_thread_.data_queue_.empty()) {
    auto data = imu_thread_.pop();
    if(imu_data_.find(data) != imu_data_.end()) {
      sensor_msgs::msg::Imu imu_msg = imu_data_[data];
      // bag_writer.write(imu_msg, "/imu/data_raw", rclcpp::Time(imu_msg.header.stamp));
      // 序列化数据
      rclcpp::Serialization<sensor_msgs::msg::Imu> serializer;
      rclcpp::SerializedMessage serialized_msg;
      serializer.serialize_message(&imu_msg, &serialized_msg);

      auto bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
      bag_message->topic_name = "/livox/imu";
      auto serialized_data = std::make_shared<rcutils_uint8_array_t>();
      serialized_data->buffer = serialized_msg.get_rcl_serialized_message().buffer;
      serialized_data->buffer_length = serialized_msg.get_rcl_serialized_message().buffer_length;
      bag_message->serialized_data = serialized_data;

      if (writer_) {
          writer_->write(bag_message);
      }
    }
  }  
  RCLCPP_INFO(this->get_logger(), "Finished writing IMU data.");
}

void read_and_write_gps_data_to_bag() {
    FILE *fp = fopen(gps_csv_path_.c_str(), "r");
    if (!fp) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open GPS data file: %s", gps_csv_path_.c_str());
        return;
    }

    int64_t stamp;
    double latitude, longitude, altitude;
    double cov[9];
    sensor_msgs::msg::NavSatFix gps_data;

    gps_data_.clear();
    while( fscanf(fp,"%ld,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",
                  &stamp,&latitude,&longitude,&altitude,&cov[0],&cov[1],&cov[2],&cov[3],&cov[4],&cov[5],&cov[6],&cov[7],&cov[8])
          == 13
          )
    {
      gps_data.header.stamp = rclcpp::Time(stamp);
      gps_data.header.frame_id = "gps";
      gps_data.latitude = latitude;
      gps_data.longitude = longitude;
      gps_data.altitude = altitude;
      for(int i = 0 ; i < 9 ; i ++) gps_data.position_covariance[i] = cov[i];
      gps_data_[stamp] = gps_data;
    }

    while(!gps_thread_.data_queue_.empty()){
    auto data = gps_thread_.pop();
    if(gps_data_.find(data) != gps_data_.end()){
      sensor_msgs::msg::NavSatFix gps_msg = gps_data_[data];
      // bag_writer.write(gps_msg, "/gps/fix", rclcpp::Time(gps_msg.header.stamp));
      rclcpp::Serialization<sensor_msgs::msg::NavSatFix> serializer;
      rclcpp::SerializedMessage serialized_msg;
      serializer.serialize_message(&gps_data, &serialized_msg);

      auto bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
      bag_message->topic_name = "/gps/fix";
      auto serialized_data = std::make_shared<rcutils_uint8_array_t>();
      serialized_data->buffer = serialized_msg.get_rcl_serialized_message().buffer;
      serialized_data->buffer_length = serialized_msg.get_rcl_serialized_message().buffer_length;
      bag_message->serialized_data = serialized_data;
      if (writer_) {
          writer_->write(bag_message);
      }

    }

  }
  fclose(fp);
  RCLCPP_INFO(this->get_logger(), "Finished writing GPS data.");
}



void read_and_write_ouster_data_to_bag()
{
  ouster_file_list_.clear();
  std::vector<std::string> tmp_files;
  struct dirent **namelist;
  int n;
  n = scandir(ouster_folder_path_.c_str(),&namelist, 0 , alphasort);
  if (n < 0)
  {
    std::string errmsg{(std::string{"No directory ("} + ouster_folder_path_ + std::string{")"})};
    const char * ptr_errmsg = errmsg.c_str();
    perror(ptr_errmsg);
  } else {
    while (n--) {
      if(std::string(namelist[n]->d_name) != "." && std::string(namelist[n]->d_name) != "..")
      {
        tmp_files.push_back(std::string(namelist[n]->d_name));
      }
      free(namelist[n]);
    }
    free(namelist);
  }

  for(auto iter = tmp_files.rbegin() ; iter!= tmp_files.rend() ; iter++) {
    ouster_file_list_.push_back(*iter);
  }

  int current_file_index = 0;
  int previous_file_index = 0;
  RCLCPP_INFO(this->get_logger(), "Ouster file list: %s", std::to_string(ouster_file_list_).c_str());
  RCLCPP_INFO(this->get_logger(), "Please Wait for a long time to Write LIDAR DATA~~~!");
  while(!ouster_thread_.data_queue_.empty()) {
    auto data = ouster_thread_.pop();

    if(std::to_string(data) + ".bin" == ouster_next_.first) {
      ouster_next_.second.header.stamp = rclcpp::Time(data);
      ouster_next_.second.header.frame_id = "ouster"; // frame ID
      // bag_writer.write(ouster_next_.second, "/raw_points", rclcpp::Time(data));
      std::shared_ptr<sensor_msgs::msg::PointCloud2> msg_ptr = std::make_shared<sensor_msgs::msg::PointCloud2>(ouster_next_.second);
      convert_to_livox_ros_driver2_CustomMsg(msg_ptr);


      rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serializer;
      rclcpp::SerializedMessage serialized_msg;
      serializer.serialize_message(&ouster_next_.second, &serialized_msg);
      // Create a bag message and write it to the ROS bag
      auto bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
      bag_message->topic_name = "/raw_points";
      auto serialized_data = std::make_shared<rcutils_uint8_array_t>();
      serialized_data->buffer = serialized_msg.get_rcl_serialized_message().buffer;
      serialized_data->buffer_length = serialized_msg.get_rcl_serialized_message().buffer_length;
      bag_message->serialized_data = serialized_data;
      writer_->write(bag_message);
      // RCLCPP_INFO(this->get_logger(), "One LiDAR frame is written to the bag.");

    } else {
      pcl::PointCloud<PointXYZIRT> cloud;
      cloud.clear();
      sensor_msgs::msg::PointCloud2 publish_cloud;
      std::string current_file_name = ouster_folder_path_ +"/"+ std::to_string(data) + ".bin";

      if(std::find(std::next(ouster_file_list_.begin(),std::max(0,previous_file_index-search_bound_)),ouster_file_list_.end(),std::to_string(data)+".bin") != ouster_file_list_.end())
      {
        std::ifstream file;
        file.open(current_file_name, std::ios::in|std::ios::binary);
        int k = 0;
        while(!file.eof())
        {
          PointXYZIRT point;
          file.read(reinterpret_cast<char *>(&point.x), sizeof(float));
          file.read(reinterpret_cast<char *>(&point.y), sizeof(float));
          file.read(reinterpret_cast<char *>(&point.z), sizeof(float));
          file.read(reinterpret_cast<char *>(&point.intensity), sizeof(float));
          point.ring = (k%64);
          k = k+1 ;
          cloud.points.push_back (point);
        }
        file.close();

        pcl::toROSMsg(cloud, publish_cloud);
        publish_cloud.header.stamp = rclcpp::Time(data);
        publish_cloud.header.frame_id = "ouster";
        // bag_writer.write(publish_cloud, "/os1_points", rclcpp::Time(data));

        std::shared_ptr<sensor_msgs::msg::PointCloud2> msg_ptr = std::make_shared<sensor_msgs::msg::PointCloud2>(publish_cloud);
        convert_to_livox_ros_driver2_CustomMsg(msg_ptr);

        rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serializer;
        rclcpp::SerializedMessage serialized_msg;
        serializer.serialize_message(&publish_cloud, &serialized_msg);

        // Create a bag message and write it to the ROS bag
        auto bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
        bag_message->topic_name = "/raw_points";
        auto serialized_data = std::make_shared<rcutils_uint8_array_t>();
        serialized_data->buffer = serialized_msg.get_rcl_serialized_message().buffer;
        serialized_data->buffer_length = serialized_msg.get_rcl_serialized_message().buffer_length;
        bag_message->serialized_data = serialized_data;

        writer_->write(bag_message);

        // RCLCPP_INFO(this->get_logger(), "One LiDAR frame is written to the bag.");
      }
      previous_file_index = 0;
    }

    //load next data
    pcl::PointCloud<PointXYZIRT> cloud;
    cloud.clear();
    sensor_msgs::msg::PointCloud2 publish_cloud;
    current_file_index = std::find(std::next(ouster_file_list_.begin(),std::max(0,previous_file_index-search_bound_)),ouster_file_list_.end(),std::to_string(data)+".bin") - ouster_file_list_.begin();
    if(std::find(std::next(ouster_file_list_.begin(),std::max(0,previous_file_index-search_bound_)),ouster_file_list_.end(),ouster_file_list_[current_file_index+1]) != ouster_file_list_.end()){
      std::string next_file_name = ouster_folder_path_ + "/" + ouster_file_list_[current_file_index+1];

      std::ifstream file;
      file.open(next_file_name, std::ios::in|std::ios::binary);
      int k = 0;
      while(!file.eof()){
        PointXYZIRT point;
        file.read(reinterpret_cast<char *>(&point.x), sizeof(float));
        file.read(reinterpret_cast<char *>(&point.y), sizeof(float));
        file.read(reinterpret_cast<char *>(&point.z), sizeof(float));
        file.read(reinterpret_cast<char *>(&point.intensity), sizeof(float));
        point.ring = (k%64);
        k = k+1 ;
        cloud.points.push_back (point);
      }

      file.close();
      pcl::toROSMsg(cloud, publish_cloud);
      ouster_next_ = make_pair(ouster_file_list_[current_file_index+1], publish_cloud);
    }
    previous_file_index = current_file_index;
  }
  RCLCPP_INFO(this->get_logger(), "Finished writing LiDAR data.");
}

void convert_to_livox_ros_driver2_CustomMsg(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    if (msg->data.empty()) {
        RCLCPP_WARN(this->get_logger(), "Received empty PointCloud2 message.");
        return;
    }
    // Create an empty CustomMsg
    auto custom_msg = livox_ros_driver2::msg::CustomMsg();
    custom_msg.header = msg->header;
    custom_msg.timebase = msg->header.stamp.sec * 1e9 + msg->header.stamp.nanosec; // Time in nanoseconds
    custom_msg.lidar_id = 0;  // Set according to your device
    custom_msg.point_num = msg->width;  // Number of points

    const uint8_t* data_ptr = msg->data.data();  // Pointer to the raw data
    std::vector<livox_ros_driver2::msg::CustomPoint> custom_points;
    custom_points.reserve(msg->width);  // Reserve space to avoid reallocations

    // Iterate through the PointCloud2 data and convert it to CustomPoint
    for (unsigned int i = 0; i < msg->width; ++i) {
        const uint8_t* point_ptr = data_ptr + i * msg->point_step;

        float x, y, z, intensity;
        memcpy(&x, point_ptr + msg->fields[0].offset, sizeof(float));  // Extract x
        memcpy(&y, point_ptr + msg->fields[1].offset, sizeof(float));  // Extract y
        memcpy(&z, point_ptr + msg->fields[2].offset, sizeof(float));  // Extract z
        memcpy(&intensity, point_ptr + msg->fields[3].offset, sizeof(float));  // Extract intensity

        intensity = std::clamp(intensity, 0.0f, 255.0f);  // Clamp intensity values to [0, 255]

        // Create a CustomPoint and set the values
        livox_ros_driver2::msg::CustomPoint custom_point;
        custom_point.x = x;
        custom_point.y = y;
        custom_point.z = z;
        custom_point.reflectivity = static_cast<uint8_t>(intensity + 0.5f);  // Round intensity
        custom_point.offset_time = 0;  // Optional: Compute offset time if needed
        custom_point.tag = 0;  // Default tag value (can be modified based on your use case)
        custom_point.line = 0;  // Default line value (can be modified based on your use case)

        // Add the custom point to the list
        custom_points.push_back(custom_point);
    }

    // Set the points in the CustomMsg
    custom_msg.points = std::move(custom_points);

    // Publish or write to ROS bag
    writeToBag(custom_msg);
}

void writeToBag(const livox_ros_driver2::msg::CustomMsg& custom_msg) {
    // Serialize the CustomMsg
    rclcpp::Serialization<livox_ros_driver2::msg::CustomMsg> serializer;
    rclcpp::SerializedMessage serialized_msg;
    serializer.serialize_message(&custom_msg, &serialized_msg);

    // Create a bag message and write it to the ROS bag
    auto bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
    bag_message->topic_name = "/livox/lidar";
    auto serialized_data = std::make_shared<rcutils_uint8_array_t>();
    serialized_data->buffer = serialized_msg.get_rcl_serialized_message().buffer;
    serialized_data->buffer_length = serialized_msg.get_rcl_serialized_message().buffer_length;
    bag_message->serialized_data = serialized_data;

    writer_->write(bag_message);
    RCLCPP_INFO(this->get_logger(), "One LiDAR frame (livox format) is written to the bag.");
}

private:
  std::string input_folder_;
  std::string output_bag_prefix_;
  std::string imu_csv_path_;
  std::string gps_csv_path_;
  std::string data_stamp_csv_path_;
  std::string ouster_folder_path_;
  std::string output_bag_path_;
  
  std::map<int64_t, std::string> data_stamp_;
  std::map<int64_t, sensor_msgs::msg::NavSatFix> gps_data_;
  std::map<int64_t, sensor_msgs::msg::Imu> imu_data_;
  std::vector<std::string> ouster_file_list_;
  std::unique_ptr<rosbag2_cpp::writers::SequentialWriter> writer_;

  DataThread<int64_t> data_stamp_thread_;
  DataThread<int64_t> gps_thread_;
  DataThread<int64_t> imu_thread_;
  DataThread<int64_t> radarpolar_thread_; 
  DataThread<int64_t> ouster_thread_;

  std::pair<std::string,sensor_msgs::msg::PointCloud2> ouster_next_;
  int search_bound_;
  int imu_data_version_; 
};




int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin_some(std::make_shared<CsvToBag>());
    rclcpp::shutdown();
    return 0;
}