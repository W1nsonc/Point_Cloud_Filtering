#include <iostream>
#include <pcl/io/pcd_io.h>
#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
 

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
    // creating a pointer that points to a memory block allocated for a pcl::PointCloud<pcl::PointXYZ> class
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    
    // convert incoming sensor_msgs::PointCloud2 messages to pcl::PointCloud<pcl::PointXYZ> type
    pcl::fromROSMsg(*input, *cloud);

    // create a pcdl::PCWriter object;
    pcl::PCDWriter writer;
    // use the write() method to save the point cloud data into a PCD file at the specified location
    writer.write("/home/cuongngo/winson_pcl_filter/src/my_pcl/PCD files/Point_cloud12.pcd", *cloud);

}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "sub_node_write_to_pcd");
    ros::NodeHandle nh;

    // Create a ROS subscriber that subscribes to a topic and that calls a callback function for each incoming ros message
    ros::Subscriber sub = nh.subscribe ("/rslidar_points", 1, cloud_cb);

    // Spin
    ros::spin ();
}