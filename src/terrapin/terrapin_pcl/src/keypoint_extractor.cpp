#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>


#include <pcl/io/pcd_io.h>
#include <pcl/common/io.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/features/normal_3d.h>

#include "std_msgs/String.h"
#include <sstream>


ros::Publisher msgPub;
ros::Publisher keypointPub;
typedef pcl::PointXYZ PointType;

// Parameters for sift computation
const float min_scale = 0.01f;
const int n_octaves = 3;
const int n_scales_per_octave = 4;
const float min_contrast = 0.001f;


void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
  // Container for original & keypoint data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 keypoints;

  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);


  // Log input size
  std_msgs::String msg1;
  std::stringstream ss1;
  ss1 << "Input   Height: " << cloud->height << "  width: " << cloud->width;
  msg1.data = ss1.str();

  msgPub.publish (msg1);


  // Extract keypoints

  // Convert to PointXYZ cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(*cloud, *cloud_xyz);

  // Estimate the normals of the cloud_xyz
  pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> ne;
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals (new pcl::PointCloud<pcl::PointNormal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_n(new pcl::search::KdTree<pcl::PointXYZ>());

  ne.setInputCloud(cloud_xyz);
  ne.setSearchMethod(tree_n);
  ne.setRadiusSearch(0.2);
  ne.compute(*cloud_normals);

  // Copy the xyz info from cloud_xyz and add it to cloud_normals as the xyz field in PointNormals estimation is zero
  for(size_t i = 0; i<cloud_normals->points.size(); ++i)
  {
    cloud_normals->points[i].x = cloud_xyz->points[i].x;
    cloud_normals->points[i].y = cloud_xyz->points[i].y;
    cloud_normals->points[i].z = cloud_xyz->points[i].z;
  }

  // Estimate the sift interest points using normals values from xyz as the Intensity variants
  pcl::SIFTKeypoint<pcl::PointNormal, pcl::PointWithScale> sift;
  pcl::PointCloud<pcl::PointWithScale> result;
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal> ());
  sift.setSearchMethod(tree);
  sift.setScales(min_scale, n_octaves, n_scales_per_octave);
  sift.setMinimumContrast(min_contrast);
  sift.setInputCloud(cloud_normals);
  sift.compute(result);



  // Convert to ROS data type
  pcl::toPCLPointCloud2(result, keypoints);
  sensor_msgs::PointCloud2 output;
  pcl_conversions::fromPCL(keypoints, output);
  output.header.frame_id = "camera_link";

  // Publish the data
  std_msgs::String msg;
  std::stringstream ss2;
  ss2 << "Results: " << result.points.size();
  msg.data = ss2.str();
  msgPub.publish (msg);

  std::stringstream ss3;
  ss3 << "Output    Height: " << keypoints.height << "   Width: " << keypoints.width;
  msg.data = ss3.str();
  msgPub.publish (msg);

  keypointPub.publish (output);
}


int main (int argc, char** argv) {
  // Initialize ROS
  ros::init (argc, argv, "terrapin_pcl");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  msgPub = nh.advertise<std_msgs::String> ("point_clouds/message_keypoints", 1000);
  keypointPub = nh.advertise<sensor_msgs::PointCloud2> ("point_clouds/keypoints", 50);

  // Spin
  ros::spin ();
}
