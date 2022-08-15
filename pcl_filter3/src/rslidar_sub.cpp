#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>    // This header allows you to publish and subscribe pcl::PointCloud objects as ROS messages
#include <pcl/point_types.h>        // The 3 includes are package depencies that pcl_filter2 needs. Need to define in CMakeLists.txt
#include <boost/foreach.hpp>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;  // aliasing the pcl's library's class PointCloud with templated data type <pcl::PointXYZ>
                                                    // to the new name PointCloud
void callback(const PointCloud::ConstPtr& msg)      // creating a function that takes in a constant object called msg of the class in typedef
{                                                   // and this object is a constant pointer
  printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
  //printf("hello");
  BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points)
    printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sub_pcl");                 //node name is sub_pcl
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<PointCloud>("rslidar_points", 1000, callback);   //subscribe to topic rslidar_points, queue size 1000,
  ros::spin();                                                                     //call the function callback each time a new message is received
}