#ifndef ROSBAG_HANDLER_HPP_
#define ROSBAG_HANDLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "autoware_sensing_msgs/msg/gnss_ins_orientation_stamped.hpp"
#include <memory>

class RosbagHandler : public rclcpp::Node
{
public:
    RosbagHandler();

private:
    void process_message(const std::shared_ptr<rosbag2_storage::SerializedBagMessage> &bag_message);

    template<typename MessageType>
    std::shared_ptr<MessageType> deserialize_message(const std::shared_ptr<rosbag2_storage::SerializedBagMessage> &bag_message);

    template<typename MessageType>
    void serialize_and_write_message(const std::string &topic_name, const std::shared_ptr<MessageType> &message, const rclcpp::Time &timestamp);

    void handle_pointcloud(const std::shared_ptr<sensor_msgs::msg::PointCloud2> &msg, const rclcpp::Time &timestamp);
    void handle_nav_sat_fix(const std::shared_ptr<sensor_msgs::msg::NavSatFix> &msg, const rclcpp::Time &timestamp);
    void handle_tf_static(const std::shared_ptr<tf2_msgs::msg::TFMessage> &msg, const rclcpp::Time &timestamp);
    void handle_twist_with_covariance(const std::shared_ptr<geometry_msgs::msg::TwistWithCovarianceStamped> &msg, const rclcpp::Time &timestamp);
    void handle_clock(const std::shared_ptr<rosgraph_msgs::msg::Clock> &msg, const rclcpp::Time &timestamp);
    void handle_odom(const std::shared_ptr<nav_msgs::msg::Odometry> &msg, const rclcpp::Time &timestamp);
    void handle_imu(const std::shared_ptr<sensor_msgs::msg::Imu> &msg, const rclcpp::Time &timestamp);
    void handle_autoware_orientation(const std::shared_ptr<autoware_sensing_msgs::msg::GnssInsOrientationStamped> &msg, const rclcpp::Time &timestamp);

    std::unique_ptr<rosbag2_cpp::Reader> reader_;
    std::unique_ptr<rosbag2_cpp::Writer> writer_;
};

#endif // ROSBAG_HANDLER_HPP_
