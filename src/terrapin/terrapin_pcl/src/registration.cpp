#include <sstream>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// #include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/keypoints/iss_3d.h>
// #include <pcl/keypoints/sift_keypoint.h>
// #include <pcl/keypoints/uniform_sampling.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation_svd.h>

using namespace std;
using namespace ros;
using namespace sensor_msgs;
using namespace pcl;
using namespace pcl::registration;
using namespace pcl_conversions;

// ********************** GLOBALS ***********************************

// Ros Publishers
// String messages
Publisher regMsgPub;
stringstream ssMsg;
std_msgs::String rosMsg;
// PointCloud2 data
Publisher originalCloudPub;
Publisher filteredCloudPub;
Publisher currentKeypointsPub;
Publisher previousKeypointsPub;
// Velocity data
Publisher imuVelocityPub;
Publisher calculatedVelocityPub;

// Variables
const float filter_density = 0.01;
const int keypoint_threshold = 25;
const double cloud_resolution = 0.006;
// SIFT Parameters
// const float min_scale = 0.01f;
// const int n_octaves = 3;
// const int n_scales_per_octave = 4;
// const float min_contrast = 0.001f;

// Previous Cloud Data
bool previousExists = false;
PointCloud<PointXYZRGB>::Ptr previous_keypoints (new PointCloud<PointXYZRGB>);
PointCloud<PointNormal>::Ptr previous_descriptors (new PointCloud<PointNormal>);

// PCL Operation Objects
VoxelGrid<PCLPointCloud2> voxel_grid;
ISSKeypoint3D<PointXYZRGB, PointXYZRGB> iss3d;
// SIFTKeypoint<PointXYZRGB, PointXYZRGB> sift;
// UniformSampling<PointXYZRGB> uniform;
NormalEstimation<PointXYZRGB, PointNormal> normal_est;
// FPFHEstimation<PointXYZRGB, PointNormal, FPFHSignature33> fpfh_est;
CorrespondenceEstimation<PointNormal, PointNormal> corr_est;
CorrespondenceRejectorSampleConsensus<PointXYZRGB> reject;
TransformationEstimationSVD<PointXYZRGB, PointXYZRGB> trans_est;


// ********************** FUNCTIONS ***********************************


void publishRegMsg () {
  // Publish stringstream and clear it's contents
  rosMsg.data = ssMsg.str();
  regMsgPub.publish(rosMsg);
  ssMsg.str(string());
}


void cloud_cb (const PointCloud2ConstPtr& cloud_msg) {
  // Publish the original cloud
  originalCloudPub.publish(cloud_msg);

  // FILTER THE POINT CLOUD
  PCLPointCloud2* cloud = new PCLPointCloud2;
  toPCL(*cloud_msg, *cloud);
  PCLPointCloud2ConstPtr cloudPtr (cloud);
  PCLPointCloud2 cloud_filtered;
  voxel_grid.setInputCloud(cloudPtr);
  voxel_grid.setLeafSize(filter_density, filter_density, filter_density);
  voxel_grid.filter(cloud_filtered);

  // Convert cloud to PointXYZRGB
  PointCloud<PointXYZRGB>::Ptr cloud_xyz (new PointCloud<PointXYZRGB>);
  fromPCLPointCloud2(cloud_filtered, *cloud_xyz);

  // Publish Message: Input data
  ssMsg << "INPUT    Height: " << cloud->height << "  Width: " << cloud->width;
  publishRegMsg();

  // Publish Message: Filtered cloud data
  ssMsg << "FILTERED POINTS: " << cloud_xyz->points.size();
  publishRegMsg();

  // Publish filtered point cloud
  PointCloud2 filteredOutput;
  fromPCL(cloud_filtered, filteredOutput);
  filteredOutput.header = cloud_msg->header;
  filteredCloudPub.publish(filteredOutput);


  // EXTRACT KEYPOINTS
  PointCloud<PointXYZRGB>::Ptr current_keypoints (new PointCloud<PointXYZRGB>);
  search::KdTree<PointXYZRGB>::Ptr keypoint_tree (new search::KdTree<PointXYZRGB> ());

  // ISS 3D
  iss3d.setInputCloud(cloud_xyz);
  iss3d.setSearchMethod(keypoint_tree);
  iss3d.setSalientRadius(6.0 * cloud_resolution);
  iss3d.setNonMaxRadius(4.0 * cloud_resolution);
  iss3d.setThreshold21(0.975);
  iss3d.setThreshold32(0.975);
  iss3d.setMinNeighbors(5);
  iss3d.setNumberOfThreads(1);
  iss3d.compute(*current_keypoints);

  // SIFT Feature extractor
  // sift.setInputCloud(cloud_xyz);
  // sift.setSearchMethod(keypoint_tree);
  // sift.setScales(min_scale, n_octaves, n_scales_per_octave);
  // sift.setMinimumContrast(min_contrast);
  // sift.compute(*current_keypoints);

  // Uniform Sampling
  // uniform.setInputCloud(cloud_xyz);
  // uniform.setSearchMethod(keypoint_tree);
  // uniform.setRadiusSearch(1.0); // 1m
  // uniform.compute(current_keypoints);

  // Publish Message: Keypoints
  ssMsg << "KEYPOINTS FOUND: " << current_keypoints->points.size();
  publishRegMsg();

  // Check if there are enough keypoints before continuing
  if (current_keypoints->points.size() >= keypoint_threshold) {

    // CREATE FEATURE DISCRIPTORS
    // Normal Estimation
    PointCloud<PointNormal>::Ptr current_descriptors (new PointCloud<PointNormal>);
    search::KdTree<PointXYZRGB>::Ptr tree_n (new search::KdTree<PointXYZRGB>());
    normal_est.setInputCloud(current_keypoints);
    normal_est.setSearchMethod(tree_n);
    normal_est.setRadiusSearch(0.2);
    normal_est.compute(*current_descriptors);

    // FPFH Estimation (Fast Point Feature Histograms)
    // PointCloud<PointXYZRGB>::Ptr current_keypoints_ptr (&current_keypoints);
    // PointCloud<FPFHSignature33>::Ptr current_descriptors (new PointCloud<FPFHSignature33>);
    // fpfh_est.setInputCloud(current_keypoints_ptr);
    // fpfh_est.setInputNormals(cloud_normals);
    // fpfh_est.setSearchSurface(cloud_xyz);
    // fpfh_est.setRadiusSearch(1); // 1m
    // fpfh_est.compute(*current_descriptors);

    // Check if previous cloud exists before continuing
    if (!previousExists) {

      // Publish Message: pervious cloud not found
      ssMsg << "No Previous Cloud Data Was Found...";
      publishRegMsg();

      // Set previous collections
      previous_keypoints->swap(*current_keypoints);
      previous_descriptors->swap(*current_descriptors);
      previousExists = true;

      // Publish Message: pervious cloud set
      ssMsg << "Previous Cloud Set";
      publishRegMsg();

    } else {

      // Publish Message: Previous cloud found
      ssMsg << "Previous Cloud Found!";
      publishRegMsg();

      // ESTIMATE CORRESPONDENCE
      CorrespondencesPtr all_correspondences (new Correspondences);
      corr_est.setInputSource(current_descriptors);
      corr_est.setInputTarget(previous_descriptors);
      corr_est.determineReciprocalCorrespondences(*all_correspondences);


      // CORRESPONDENCE REJECTION
      CorrespondencesPtr good_correspondences (new Correspondences);
      reject.setInputSource(current_keypoints);
      reject.setInputTarget(previous_keypoints);
      reject.setInputCorrespondences(all_correspondences);
      reject.getCorrespondences(*good_correspondences);


      // EXTIMATE TRANSFORMATION
      Eigen::Matrix4f transform;
      trans_est.estimateRigidTransformation(*current_keypoints, *previous_keypoints, *good_correspondences, transform);

      // Publish Message: Transformation
      ssMsg << "TRANSFORMATION: " << transform;
      publishRegMsg();

      // Publish keypoint point clouds
      PCLPointCloud2 currKeypoints;
      PCLPointCloud2 prevKeypoints;
      toPCLPointCloud2(*current_keypoints, currKeypoints);
      toPCLPointCloud2(*previous_keypoints, prevKeypoints);
      PointCloud2 currOutput;
      PointCloud2 prevOutput;
      fromPCL(currKeypoints, currOutput);
      fromPCL(prevKeypoints, prevOutput);
      currOutput.header = cloud_msg->header;
      prevOutput.header = cloud_msg->header;
      currentKeypointsPub.publish(currOutput);
      previousKeypointsPub.publish(prevOutput);

      // Set previous collections
      previous_keypoints->swap(*current_keypoints);
      previous_descriptors->swap(*current_descriptors);
    }
  }
  // Publish Message: Done
  ssMsg << "------------------------------------------------";
  publishRegMsg();
}


int main (int argc, char** argv) {
  // Initialize ROS
  ros::init (argc, argv, "terrapin_pcl");
  NodeHandle nh;

  // Create ROS Subscribers
  Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create ROS Publishers
  regMsgPub = nh.advertise<std_msgs::String> ("terrapin/registration/messages", 1);
  originalCloudPub = nh.advertise<PointCloud2> ("terrapin/point_clouds/original", 1);
  filteredCloudPub = nh.advertise<PointCloud2> ("terrapin/point_clouds/filtered", 1);
  currentKeypointsPub = nh.advertise<PointCloud2> ("terrapin/point_clouds/current_keypoints", 1);
  previousKeypointsPub = nh.advertise<PointCloud2> ("terrapin/point_clouds/previous_keypoints", 1);

  // Spin
  ros::spin ();
}
