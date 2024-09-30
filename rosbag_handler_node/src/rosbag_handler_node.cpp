#include "rosbag_handler_node/rosbag_handler_node.hpp"
#include <filesystem>
#include <chrono>
#include <iomanip>
#include <sstream>

RosbagHandler::RosbagHandler() : Node("rosbag_handler_node")
{
  std::string input_bag_path = "/home/melike/rosbags/urban_shared_data/rosbag2_2024_09_11-17_53_54/rosbag2_2024_09_11-17_53_54_0.db3";

  // Generate a unique output path using timestamp
  auto now = std::chrono::system_clock::now();
  auto now_c = std::chrono::system_clock::to_time_t(now);
  std::stringstream ss;
  ss << "/home/melike/rosbags/urban_shared_data/write_new_bag/test_"
     << std::put_time(std::localtime(&now_c), "%Y%m%d_%H%M%S");

  std::string output_bag_path = ss.str();

  // Setup storage options for reader
  rosbag2_storage::StorageOptions reader_storage_options;
  reader_storage_options.uri = input_bag_path;
  reader_storage_options.storage_id = "sqlite3";  // Default is sqlite3 for ROS2

  // Initialize the reader
  reader_ = std::make_unique<rosbag2_cpp::Reader>();
  try {
    reader_->open(reader_storage_options);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open input bag file: %s", e.what());
    throw;
  }

  // Initialize the writer
  writer_ = std::make_unique<rosbag2_cpp::Writer>();
  try {
    writer_->open(output_bag_path);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open output bag file: %s", e.what());
    throw;
  }

  // Get topic information from the input bag and create them in the output bag
  auto topics_and_types = reader_->get_all_topics_and_types();

  RCLCPP_INFO(this->get_logger(), "Topics in the input bag:");
  for (const auto &topic_info : topics_and_types) {
    RCLCPP_INFO(this->get_logger(), "Topic: %s, Type: %s", topic_info.name.c_str(), topic_info.type.c_str());

    // Only create topics we are interested in
    if (topic_info.name == "/pandar_points" ||
        topic_info.name == "/applanix/lvx_client/gnss/fix" ||
        topic_info.name == "/tf_static" ||
        topic_info.name == "/localization/twist_estimator/twist_with_covariance" ||
        topic_info.name == "/clock" ||
        topic_info.name == "/applanix/lvx_client/odom" ||
        topic_info.name == "/applanix/lvx_client/imu_raw" ||
        topic_info.name == "/applanix/lvx_client/autoware_orientation") {
      writer_->create_topic(topic_info);
    }
  }

  while (reader_->has_next()) {
    auto bag_message = reader_->read_next();
    process_message(bag_message);
  }

  RCLCPP_INFO(this->get_logger(), "Finished processing bag files.");
}

void RosbagHandler::process_message(const std::shared_ptr<rosbag2_storage::SerializedBagMessage> &bag_message)
{
  std::string topic_name = bag_message->topic_name;
  rclcpp::Time timestamp(bag_message->time_stamp);

  if (topic_name == "/pandar_points") {
    auto pointcloud_msg = deserialize_message<sensor_msgs::msg::PointCloud2>(bag_message);
    handle_pointcloud(pointcloud_msg, timestamp);
  } else if (topic_name == "/applanix/lvx_client/gnss/fix") {
    auto nav_sat_fix_msg = deserialize_message<sensor_msgs::msg::NavSatFix>(bag_message);
    handle_nav_sat_fix(nav_sat_fix_msg, timestamp);
  } else if (topic_name == "/tf_static") {
    auto tf_static_msg = deserialize_message<tf2_msgs::msg::TFMessage>(bag_message);
    handle_tf_static(tf_static_msg, timestamp);
  } else if (topic_name == "/localization/twist_estimator/twist_with_covariance") {
    auto twist_msg = deserialize_message<geometry_msgs::msg::TwistWithCovarianceStamped>(bag_message);
    handle_twist_with_covariance(twist_msg, timestamp);
  } else if (topic_name == "/clock") {
    auto clock_msg = deserialize_message<rosgraph_msgs::msg::Clock>(bag_message);
    handle_clock(clock_msg, timestamp);
  } else if (topic_name == "/applanix/lvx_client/odom") {
    auto odom_msg = deserialize_message<nav_msgs::msg::Odometry>(bag_message);
    handle_odom(odom_msg, timestamp);
  } else if (topic_name == "/applanix/lvx_client/imu_raw") {
    auto imu_msg = deserialize_message<sensor_msgs::msg::Imu>(bag_message);
    handle_imu(imu_msg, timestamp);
  } else if (topic_name == "/applanix/lvx_client/autoware_orientation") {
    auto autoware_orientation_msg = deserialize_message<autoware_sensing_msgs::msg::GnssInsOrientationStamped>(bag_message);
    handle_autoware_orientation(autoware_orientation_msg, timestamp);
  } else {
    // Optionally, log unhandled topics
    RCLCPP_DEBUG(this->get_logger(), "Ignoring topic: %s", topic_name.c_str());
  }

  // Removed the unconditional writer_->write(bag_message);
  // Only write messages in the handle_* functions
}

template<typename MessageType>
std::shared_ptr<MessageType> RosbagHandler::deserialize_message(const std::shared_ptr<rosbag2_storage::SerializedBagMessage> &bag_message)
{
  rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
  auto message = std::make_shared<MessageType>();
  rclcpp::Serialization<MessageType> serializer;
  serializer.deserialize_message(&serialized_msg, message.get());
  return message;
}

template<typename MessageType>
void RosbagHandler::serialize_and_write_message(const std::string &topic_name, const std::shared_ptr<MessageType> &message, const rclcpp::Time &timestamp)
{
  rclcpp::SerializedMessage serialized_msg;
  rclcpp::Serialization<MessageType> serializer;
  serializer.serialize_message(message.get(), &serialized_msg);

  auto bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
  bag_message->time_stamp = timestamp.nanoseconds();
  bag_message->topic_name = topic_name;
  bag_message->serialized_data = std::make_shared<rcutils_uint8_array_t>(serialized_msg.get_rcl_serialized_message());

  try {
    writer_->write(bag_message);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to write serialized message to output bag: %s", e.what());
  }
}

void RosbagHandler::handle_pointcloud(const std::shared_ptr<sensor_msgs::msg::PointCloud2> &msg, const rclcpp::Time &timestamp)
{
  RCLCPP_INFO(this->get_logger(), "PointCloud2 message received with width: %d, height: %d", msg->width, msg->height);

  // Optionally process the message here

  // Serialize and write the message to the output bag
  serialize_and_write_message("/pandar_points", msg, timestamp);
}

void RosbagHandler::handle_nav_sat_fix(const std::shared_ptr<sensor_msgs::msg::NavSatFix> &msg, const rclcpp::Time &timestamp)
{
  RCLCPP_INFO(this->get_logger(), "NavSatFix received: [%.2f, %.2f, %.2f]",
              msg->latitude, msg->longitude, msg->altitude);

  // Optionally process the message here

  // Serialize and write the message to the output bag
  serialize_and_write_message("/applanix/lvx_client/gnss/fix", msg, timestamp);
}

void RosbagHandler::handle_tf_static(const std::shared_ptr<tf2_msgs::msg::TFMessage> &msg, const rclcpp::Time &timestamp)
{
  for (const auto &transform : msg->transforms) {
    RCLCPP_INFO(this->get_logger(), "Static transform received from %s to %s",
                transform.header.frame_id.c_str(), transform.child_frame_id.c_str());
  }

  // Optionally process the message here

  // Serialize and write the message to the output bag
  serialize_and_write_message("/tf_static", msg, timestamp);
}

void RosbagHandler::handle_twist_with_covariance(const std::shared_ptr<geometry_msgs::msg::TwistWithCovarianceStamped> &msg, const rclcpp::Time &timestamp)
{
  RCLCPP_INFO(this->get_logger(), "TwistWithCovarianceStamped received: [linear.x: %.2f, angular.z: %.2f]",
              msg->twist.twist.linear.x, msg->twist.twist.angular.z);

  // Optionally process the message here

  // Serialize and write the message to the output bag
  serialize_and_write_message("/localization/twist_estimator/twist_with_covariance", msg, timestamp);
}

void RosbagHandler::handle_clock(const std::shared_ptr<rosgraph_msgs::msg::Clock> &msg, const rclcpp::Time &timestamp)
{
  RCLCPP_INFO(this->get_logger(), "Clock message received: %d", msg->clock.sec);

  // Optionally process the message here

  // Serialize and write the message to the output bag
  serialize_and_write_message("/clock", msg, timestamp);
}

void RosbagHandler::handle_odom(const std::shared_ptr<nav_msgs::msg::Odometry> &msg, const rclcpp::Time &timestamp)
{
  RCLCPP_INFO(this->get_logger(), "Odometry message received: [%.2f, %.2f, %.2f]",
              msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);

  // Optionally process the message here

  // Serialize and write the message to the output bag
  serialize_and_write_message("/applanix/lvx_client/odom", msg, timestamp);
}

void RosbagHandler::handle_imu(const std::shared_ptr<sensor_msgs::msg::Imu> &msg, const rclcpp::Time &timestamp)
{
  RCLCPP_INFO(this->get_logger(), "Imu message received: [%.2f, %.2f, %.2f]",
              msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);

  // Serialize and write the message to the output bag
  serialize_and_write_message("/applanix/lvx_client/imu_raw", msg, timestamp);
}

void RosbagHandler::handle_autoware_orientation(const std::shared_ptr<autoware_sensing_msgs::msg::GnssInsOrientationStamped> &msg, const rclcpp::Time &timestamp)
{
  RCLCPP_INFO(this->get_logger(), "Autoware orientation message received: [%.2f, %.2f, %.2f]",
              msg->orientation.rmse_rotation_x, msg->orientation.rmse_rotation_y, msg->orientation.rmse_rotation_z);

  // Optionally process the message here

  // Serialize and write the message to the output bag
  serialize_and_write_message("/applanix/lvx_client/autoware_orientation", msg, timestamp);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RosbagHandler>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
