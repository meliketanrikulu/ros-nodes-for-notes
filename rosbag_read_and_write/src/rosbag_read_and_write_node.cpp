#include "rosbag_read_and_write/rosbag_read_and_write.h"

#include "ros/ros.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH


RosbagReadAndWriteClass::RosbagReadAndWriteClass(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{

    bag.open("indoor_2.bag", rosbag::bagmode::Read);
    new_bag.open("my_bag.bag", rosbag::bagmode::Write);
   
    std::vector<std::string> topics;
    topics.push_back(std::string("/velodyne_points"));

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    foreach(rosbag::MessageInstance const m, view)
    {
        sensor_msgs::PointCloud2::ConstPtr bag_msg = m.instantiate<sensor_msgs::PointCloud2>();
        if (bag_msg != NULL){
            new_bag_msg.header = bag_msg->header;
            new_bag_msg.height = bag_msg->height;
            new_bag_msg.width = bag_msg->width;
            new_bag_msg.fields = bag_msg->fields;
            new_bag_msg.is_bigendian = bag_msg->is_bigendian;
            new_bag_msg.point_step = bag_msg->point_step;
            new_bag_msg.row_step = bag_msg->row_step;
            new_bag_msg.is_dense = bag_msg->is_dense;
            new_bag_msg.data = bag_msg->data ;
            std::cout <<"continue..." << std::endl;
            new_bag.write("my_pointCloud", new_bag_msg.header.stamp, new_bag_msg);
        }
    	else{
    	    std::cout<<"empty bag file!"<<std::endl;
    	}

    }

    std::cout<<"done!"<<std::endl;
    bag.close();
 
    new_bag.close();
  
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "RosbagReadAndWrite"); //node name
  ros::NodeHandle nh;
  RosbagReadAndWriteClass rosbagReadAndWriteClass(&nh);
  ros::spin();
  return 0;
}

