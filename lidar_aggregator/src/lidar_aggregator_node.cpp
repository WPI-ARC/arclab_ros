#include <ros/ros.h>
#include <math.h>
#include <string>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>
#include <lidar_aggregator/LidarAggregation.h>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>

std::string g_fixed_frame;
ros::Publisher g_cloud_publisher;
laser_geometry::LaserProjection g_laser_projector;
tf::TransformListener* g_transformer;

bool LaserAggregationServiceCB(lidar_aggregator::LidarAggregation::Request& req, lidar_aggregator::LidarAggregation::Response& res)
{
    ROS_INFO("Attempting to aggregate %ld laser scans into a pointcloud", req.Scans.size());
    if (req.Scans.size() > 1)
    {
        /* If there are more than one scan, then we need to carefully convert between ROS and PCL pointcloud types on the fly */
        // Process and convert the first scan
        sensor_msgs::PointCloud2 temp_ros_cloud;
        pcl::PCLPointCloud2 pcl_full_cloud;
        g_laser_projector.transformLaserScanToPointCloud(g_fixed_frame, req.Scans[0], temp_ros_cloud, *g_transformer);
        pcl_conversions::moveToPCL(temp_ros_cloud, pcl_full_cloud);
        // Process, convert, and combine together the remaining scans
        for (unsigned int index = 1; index < req.Scans.size(); index++)
        {
            g_laser_projector.transformLaserScanToPointCloud(g_fixed_frame, req.Scans[index], temp_ros_cloud, *g_transformer);
            pcl::PCLPointCloud2 temp_pcl_cloud;
            pcl_conversions::moveToPCL(temp_ros_cloud, temp_pcl_cloud);
            bool succeded = pcl::concatenatePointCloud(pcl_full_cloud, temp_pcl_cloud, pcl_full_cloud);
            if (!succeded)
            {
                ROS_ERROR("PCL could not concatenate pointclouds");
            }
        }
        // Convert the PCL pointcloud to ROS and finish
        sensor_msgs::PointCloud2 ros_cloud;
        pcl_conversions::moveFromPCL(pcl_full_cloud, ros_cloud);
        ros_cloud.header.frame_id = g_fixed_frame;
        ros_cloud.header.stamp = ros::Time::now();
        res.Cloud = ros_cloud;
        g_cloud_publisher.publish(ros_cloud);
        return true;
    }
    else if (req.Scans.size() == 1)
    {
        /* If there is exactly one scan, we don't need to do any conversions beteen ROS and PCL explicitly */
        sensor_msgs::PointCloud2 ros_cloud;
        g_laser_projector.transformLaserScanToPointCloud(g_fixed_frame, req.Scans[0], ros_cloud, *g_transformer);
        ros_cloud.header.frame_id = g_fixed_frame;
        ros_cloud.header.stamp = ros::Time::now();
        res.Cloud = ros_cloud;
        g_cloud_publisher.publish(ros_cloud);
        return true;
    }
    else
    {
        /* If there are no scans at all, warn and return an empty cloud */
        ROS_WARN("No laser scans to aggregate");
        sensor_msgs::PointCloud2 ros_cloud;
        ros_cloud.header.frame_id = g_fixed_frame;
        ros_cloud.header.stamp = ros::Time::now();
        res.Cloud = ros_cloud;
        g_cloud_publisher.publish(ros_cloud);
        return true;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_aggregator");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    tf::TransformListener listener(nh, ros::Duration(20.0));
    g_transformer = &listener;
    ROS_INFO("Starting LIDAR aggregator...");
    nhp.param(std::string("fixed_frame"), g_fixed_frame, std::string("torso_lift_link"));
    g_cloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("pointcloud", 1, true);
    ros::ServiceServer server = nh.advertiseService("aggregate_lidar", LaserAggregationServiceCB);
    ROS_INFO("LIDAR aggregator loaded");
    while (ros::ok())
    {
        ros::spinOnce();
    }
    ROS_INFO("Shutting down LIDAR aggregator");
    return 0;
}
