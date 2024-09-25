#include "rclcpp/rclcpp.hpp"
#include "rosbag_parser_node/rosbag_parser_node.hpp"
#include <memory>

RosbagParserNode::RosbagParserNode() : Node("rosbag_parser_node") {
    pc_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pc_topic", 10);
    pose_with_cov_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("pose_topic", 10);
    twist_with_cov_publisher_ = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("twist_topic", 10);
    tf_publisher_ = this->create_publisher<tf2_msgs::msg::TFMessage>("tf_topic", 10);


    std::string bag_file_path = "/home/melike/rosbags/urban_shared_data/only_localization_test/rosbag2_2024_09_12-14_59_58/rosbag2_2024_09_12-14_59_58_0.db3"; // Update with actual path

    // Setup storage options
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = bag_file_path;
    storage_options.storage_id = "sqlite3";  // Default is sqlite3 for ROS2

    // Setup converter options
    rosbag2_cpp::ConverterOptions converter_options;
    converter_options.input_serialization_format = "cdr";
    converter_options.output_serialization_format = "cdr";

    // Initialize the reader
    reader_ = std::make_unique<rosbag2_cpp::Reader>();
    reader_->open(storage_options, converter_options);

    while (reader_->has_next()) {
        auto bag_message = reader_->read_next();
        process_message(bag_message);
    }

}

void RosbagParserNode::process_message(const std::shared_ptr<rosbag2_storage::SerializedBagMessage> &bag_message)
{
  std::string topic_name = bag_message->topic_name;

  if (topic_name == "/localization/util/downsample/pointcloud") {
    auto pointcloud_msg = deserialize_message<sensor_msgs::msg::PointCloud2>(bag_message);
    handle_pointcloud(pointcloud_msg);
  } else if (topic_name == "/sensing/gnss/pose_with_covariance") {
    auto pose_msg = deserialize_message<geometry_msgs::msg::PoseWithCovarianceStamped>(bag_message);
    handle_pose_with_covariance(pose_msg);
  } else if (topic_name == "/tf_static") {
    auto tf_static_msg = deserialize_message<tf2_msgs::msg::TFMessage>(bag_message);
    handle_tf_static(tf_static_msg);
  } else if (topic_name == "/localization/twist_estimator/twist_with_covariance") {
    auto twist_msg = deserialize_message<geometry_msgs::msg::TwistWithCovarianceStamped>(bag_message);
    handle_twist_with_covariance(twist_msg);
  }
}

template<typename MessageType>
std::shared_ptr<MessageType> RosbagParserNode::deserialize_message(const std::shared_ptr<rosbag2_storage::SerializedBagMessage> &bag_message)
{
  rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
  auto message = std::make_shared<MessageType>();
  rclcpp::Serialization<MessageType> serializer;
  serializer.deserialize_message(&serialized_msg, message.get());
  return message;
}

void RosbagParserNode::handle_pointcloud(const std::shared_ptr<sensor_msgs::msg::PointCloud2> &msg)
{
  RCLCPP_INFO(this->get_logger(), "PointCloud2 message received with width: %d, height: %d", msg->width, msg->height);
  sensor_msgs::msg::PointCloud2 pc_msg = *msg;
  pc_publisher_->publish(pc_msg);
}

void RosbagParserNode::handle_pose_with_covariance(const std::shared_ptr<geometry_msgs::msg::PoseWithCovarianceStamped> &msg)
{
  RCLCPP_INFO(this->get_logger(), "PoseWithCovarianceStamped received: [%.2f, %.2f, %.2f]", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
  geometry_msgs::msg::PoseWithCovarianceStamped pose_msg = *msg;
  pose_with_cov_publisher_->publish(pose_msg);
}

void RosbagParserNode::handle_tf_static(const std::shared_ptr<tf2_msgs::msg::TFMessage> &msg)
{
  for (const auto &transform : msg->transforms) {
    RCLCPP_INFO(this->get_logger(), "Static transform received from %s to %s", transform.header.frame_id.c_str(), transform.child_frame_id.c_str());
    tf2_msgs::msg::TFMessage tf_msg = *msg;
    tf_publisher_->publish(tf_msg);
  }
}

void RosbagParserNode::handle_twist_with_covariance(const std::shared_ptr<geometry_msgs::msg::TwistWithCovarianceStamped> &msg)
{
  RCLCPP_INFO(this->get_logger(), "TwistWithCovarianceStamped received: [linear.x: %.2f, angular.z: %.2f]", msg->twist.twist.linear.x, msg->twist.twist.angular.z);
  geometry_msgs::msg::TwistWithCovarianceStamped twist_msg = *msg;
  twist_with_cov_publisher_->publish(twist_msg);
}


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RosbagParserNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


