#ifndef EUCLIDEAN_CLUSTERING_HPP
#define EUCLIDEAN_CLUSTERING_HPP

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>

class EuclideanClustering {
public:
    EuclideanClustering() ;
    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher clustered_cloud_pub_;
    ros::Publisher downsampled_cloud_pub_;
    ros::Publisher non_clustered_cloud_pub_;

    double z_min_;
    double z_max_;
    double y_min_;
    double y_max_;
    double downsample_leaf_size_;
    double cluster_tolerance_;
    int min_cluster_size_;
    int max_cluster_size_;
};

#endif // EUCLIDEAN_CLUSTERING_HPP

