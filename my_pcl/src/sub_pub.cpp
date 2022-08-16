#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

void voxel_filter (const sensor_msgs::PointCloud2ConstPtr&);
//void plane_segmentation (const pcl::PCLPointCloud2ConstPtr&);
ros::Publisher pub;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{

  voxel_filter(input);

  //plane_segmentation(cloud_filtered);

}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pcl_sub_pub_node");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/rslidar_points", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("point_cloud_output", 1);

  // Spin
  ros::spin ();
}


 void voxel_filter (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
 {
  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;

  // Convert sensor_msgs/PointCloud2 message data type to PCL data type, pcl::PCLPointCloud2
  pcl_conversions::toPCL(*cloud_msg, *cloud);

  // Perform the actual filtering
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloudPtr);
  sor.setLeafSize (0.1, 0.1, 0.1);   //leaf size of 10cm x 10cm x 10cm
  sor.filter (cloud_filtered);   // outputs a PCLPointCloud2 data type

  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud< pcl::PointXYZ>); // create a pointer of pcl::PointCloud class 
  pcl::fromPCLPointCloud2(cloud_filtered, *point_cloud);  // convert the PCLPointCloud2 cloud_filtered to pclPointCloud

  // Plane segmentation
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);  //create a ModelCoefficients pointer
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices); //create a PointIndices pointer for inliers
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);
  seg.setInputCloud (point_cloud);    // takes in a const PointCloudConstPtr cloud data type argument
  seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
  }
/*      /********* This section is just to check that the plane segmentation works
  std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;

  std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
  for (std::size_t i = 0; i < inliers->indices.size (); ++i)
  for (const auto& idx: inliers->indices)
  std::cerr << idx << "    " << point_cloud->points[idx].x << " "
                             << point_cloud->points[idx].y << " "
                             << point_cloud->points[idx].z << std::endl;
*/

// Need to convert the point cloud data type from PCLPointCloud to PCL/PointCloud2 so that it can be converted to sensor_msgs/PointCloud2
//to be published


  // Convert pcl::PCLPointCloud2 data type to sensor_msgs/PointCloud2 data type
  sensor_msgs::PointCloud2 output;
  pcl_conversions::fromPCL(cloud_filtered, output);

  // Publish the data.
  pub.publish (output); 
 
}

/*
void plane_segmentation (const pcl::PCLPointCloud2ConstPtr& cloud_filtered)
{
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);  //create a ModelCoefficients pointer
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices); //create a PointIndices pointer for inliers
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);

  seg.setInputCloud (cloud_filtered);
  seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    return (-1);
  }

  // Convert to ROS data type
  sensor_msgs::PointCloud2 output;
  pcl_conversions::fromPCL(cloud_filtered, output);

  // Publish the data.
  pub.publish (output);

  }
*/




  /****************************************** Backup
  #include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

void voxel_filter (const sensor_msgs::PointCloud2ConstPtr&);
void plane_segmentation (const pcl::PointCloud2&);
ros::Publisher pub;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Create a container for the data.
  //sensor_msgs::PointCloud2 output;

  // Do data processing here...
  // output = *input
  voxel_filter(input);

  void plane_segmentation ()

  // Publish the data.
  //pub.publish (output);
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pcl_sub_pub_node");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/rslidar_points", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("point_cloud_output", 1);

  // Spin
  ros::spin ();
}


void voxel_filter (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
 // Container for original & filtered data
 pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
 pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
 pcl::PCLPointCloud2 cloud_filtered;

 // Convert sensor_msgs/PointCloud2 message data type to PCL data type, pcl::PCLPointCloud2
 pcl_conversions::toPCL(*cloud_msg, *cloud);

 // Perform the actual filtering
 pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
 sor.setInputCloud (cloudPtr);
 sor.setLeafSize (0.7, 0.1, 0.7);   //leaf size of 10cm x 10cm x 10cm
 sor.filter (cloud_filtered);

//  // Convert PCL::PointCloud2 data type to ROS data type
//  sensor_msgs::PointCloud2 output;
// pcl_conversions::fromPCL(cloud_filtered, output);

//  // Publish the data.
//  pub.publish (output);
}

void plane_segmentation (const sensor_msgs::PointCloud2ConstPtr& cloud_filtered)
{
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);  //create a ModelCoefficients pointer
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices); //create a PointIndices pointer for inliers
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);

  seg.setInputCloud (cloud_filtered);
  seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    return (-1);
  }

  // Convert pcl::PCLPointCloud2 data type to sensor_msgs/PointCloud2 data type
  sensor_msgs::PointCloud2 output;
  pcl_conversions::fromPCL(cloud_filtered, output);

  // Publish the data.
  pub.publish (output);



  }
  */