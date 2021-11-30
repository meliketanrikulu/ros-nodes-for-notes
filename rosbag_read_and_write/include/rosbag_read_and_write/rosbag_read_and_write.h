
#ifndef ROSBAG_READ_AND_WRITE___ROSBAG_READ_AND_WRITE_H_
#define ROSBAG_READ_AND_WRITE___ROSBAG_READ_AND_WRITE_H_
#include "ros/ros.h"
#include <rosbag/bag.h>
#include <sensor_msgs/PointCloud2.h>

class RosbagReadAndWriteClass
{
 public:

  RosbagReadAndWriteClass(ros::NodeHandle* nodehandle);
  ros::NodeHandle nh_;         
  sensor_msgs::PointCloud2 new_bag_msg;

  rosbag::Bag bag;
  rosbag::Bag new_bag;



};

#endif  // ROSBAG_READ_AND_WRITE___ROSBAG_READ_AND_WRITE_H_

