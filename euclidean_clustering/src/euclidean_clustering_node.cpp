#include "euclidean_clustering/euclidean_clustering.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include "pcl/filters/passthrough.h"

EuclideanClustering::EuclideanClustering() {
    sub_ = nh_.subscribe("/camera_mid/depth/color/points", 1, &EuclideanClustering::cloudCallback, this);
    downsampled_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/camera_mid/euclidean_clustering_node/downsampled_cloud", 1);
    clustered_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/camera_mid/euclidean_clustering_node/clustered_cloud", 1);
    non_clustered_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/camera_mid/euclidean_clustering_node/non_clustered_cloud", 1);

    nh_.param<double>("/euclidean_clustering_node/z_min", z_min_,0.0); 
    nh_.param<double>("/euclidean_clustering_node/z_max", z_max_,0.0);
    nh_.param<double>("/euclidean_clustering_node/y_min", y_min_,0.0);
    nh_.param<double>("/euclidean_clustering_node/y_max", y_max_,0.0);
    nh_.param<double>("/euclidean_clustering_node/downsample_leaf_size", downsample_leaf_size_,0.0);
    nh_.param<double>("/euclidean_clustering_node/cluster_tolerance", cluster_tolerance_,0.0);
    nh_.param<int>("/euclidean_clustering_node/min_cluster_size", min_cluster_size_,0);
    nh_.param<int>("/euclidean_clustering_node/max_cluster_size", max_cluster_size_,0);

    ROS_INFO("Euclidean Clustering Parameters:");
    ROS_INFO ("Z min: %f", z_min_);
    ROS_INFO("Z max: %f", z_max_);
    ROS_INFO("Y min: %f", y_min_);
    ROS_INFO("Y max: %f", y_max_);
    ROS_INFO("Downsample leaf size: %f", downsample_leaf_size_);
    ROS_INFO("Cluster tolerance: %f", cluster_tolerance_);
    ROS_INFO("Min cluster size: %d", min_cluster_size_);
    ROS_INFO("Max cluster size: %d", max_cluster_size_);


}

void EuclideanClustering::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
    // Convert ROS msg to PCL point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*cloud_msg, *cloud);

    // Remove NaN points
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *filtered_cloud, indices);

    // Filter points based on Z axis (height)
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(filtered_cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(z_min_, z_max_); // Retain points with 0 <= Z <= 1.5 meters
    pass.filter(*filtered_cloud);

    // Filter points based on Y axis (distance from origin in Y)
    pass.setInputCloud(filtered_cloud);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(y_min_, y_max_); // Retain points with -0.7 <= Y <= 0.7 meters
    pass.filter(*filtered_cloud);

    // Downsample the cloud using VoxelGrid filter
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(filtered_cloud);
    voxel_filter.setLeafSize(downsample_leaf_size_, downsample_leaf_size_, downsample_leaf_size_);
    voxel_filter.filter(*downsampled_cloud);

    sensor_msgs::PointCloud2 downsampled_msg;
    pcl::toROSMsg(*downsampled_cloud, downsampled_msg);
    downsampled_msg.header = cloud_msg->header;
    downsampled_cloud_pub_.publish(downsampled_msg);

    // KDTree for cluster extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    tree->setInputCloud(downsampled_cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(cluster_tolerance_);
    ec.setMinClusterSize(min_cluster_size_);
    ec.setMaxClusterSize(max_cluster_size_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(downsampled_cloud);
    ec.extract(cluster_indices);

    // Combine clusters into a single point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr clustered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointIndices::Ptr all_clustered_indices(new pcl::PointIndices());
    int cluster_id = 0;

    for (const auto& indices : cluster_indices) {
        for (const auto& idx : indices.indices) {
            pcl::PointXYZRGB point;
            point.x = downsampled_cloud->points[idx].x;
            point.y = downsampled_cloud->points[idx].y;
            point.z = downsampled_cloud->points[idx].z;

            // Assign a color to each cluster
            point.r = (cluster_id * 123) % 255;
            point.g = (cluster_id * 231) % 255;
            point.b = (cluster_id * 312) % 255;

            clustered_cloud->points.push_back(point);
            all_clustered_indices->indices.push_back(idx);

        }
        cluster_id++;
    }

    clustered_cloud->width = clustered_cloud->points.size();
    clustered_cloud->height = 1;
    clustered_cloud->is_dense = true;

    // Convert to ROS message and publish
    sensor_msgs::PointCloud2 output_msg;
    pcl::toROSMsg(*clustered_cloud, output_msg);
    output_msg.header = cloud_msg->header;
    clustered_cloud_pub_.publish(output_msg);


    // Extract and publish non-clustered points
    pcl::PointCloud<pcl::PointXYZ>::Ptr non_clustered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(downsampled_cloud);
    extract.setIndices(all_clustered_indices);
    extract.setNegative(true); // Extract points not in clusters
    extract.filter(*non_clustered_cloud);

    sensor_msgs::PointCloud2 non_clustered_msg;
    pcl::toROSMsg(*non_clustered_cloud, non_clustered_msg);
    non_clustered_msg.header = cloud_msg->header;
    non_clustered_cloud_pub_.publish(non_clustered_msg);
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "euclidean_clustering_node");
    EuclideanClustering ec;
    ros::spin();
    return 0;
}

