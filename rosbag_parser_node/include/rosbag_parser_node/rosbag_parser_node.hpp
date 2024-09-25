#ifndef UPDATE_PC_FIELDS__UPDATE_PC_FIELDS_HPP_
#define UPDATE_PC_FIELDS__UPDATE_PC_FIELDS_HPP_


#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <memory>
class RosbagParserNode : public rclcpp::Node {
public:
    RosbagParserNode();

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_with_cov_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_with_cov_publisher_;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_publisher_;

private:
    void process_message(const std::shared_ptr<rosbag2_storage::SerializedBagMessage> &bag_message);

    template<typename MessageType>
    std::shared_ptr<MessageType> deserialize_message(const std::shared_ptr<rosbag2_storage::SerializedBagMessage> &bag_message);

    void handle_pointcloud(const std::shared_ptr<sensor_msgs::msg::PointCloud2> &msg);
    void handle_pose_with_covariance(const std::shared_ptr<geometry_msgs::msg::PoseWithCovarianceStamped> &msg);
    void handle_tf_static(const std::shared_ptr<tf2_msgs::msg::TFMessage> &msg);
    void handle_twist_with_covariance(const std::shared_ptr<geometry_msgs::msg::TwistWithCovarianceStamped> &msg);

    std::unique_ptr<rosbag2_cpp::Reader> reader_;


};

#endif  // UPDATE_PC_FIELDS__UPDATE_PC_FIELDS_HPP_
