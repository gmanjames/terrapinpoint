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

// GLOBALS

// Ros Publishers
Publisher msgPub;
std_msgs::String msg;
Publisher keypointPub;

// Variables
const float filter_density = 0.01;
const int keypoint_threshold = 25;
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


void cloud_cb (const PointCloud2ConstPtr& cloud_msg) {
  // FILTER THE POINT CLOUD
  PCLPointCloud2* cloud = new PCLPointCloud2;
  toPCL(*cloud_msg, *cloud);
  PCLPointCloud2ConstPtr cloudPtr (cloud);
  PCLPointCloud2 cloud_filtered;
  voxel_grid.setInputCloud(cloudPtr);
  voxel_grid.setLeafSize(filter_density, filter_density, filter_density);
  voxel_grid.filter(cloud_filtered);

  // Publish Message: Input data
  stringstream inputMsg;
  inputMsg << "INPUT    Height: " << cloud->height << "  Width: " << cloud->width;
  msg.data = inputMsg.str();
  msgPub.publish (msg);

  // Convert cloud to PointXYZRGB
  PointCloud<PointXYZRGB>::Ptr cloud_xyz (new PointCloud<PointXYZRGB>);
  fromPCLPointCloud2(cloud_filtered, *cloud_xyz);
  double cloud_resolution (0.0058329);

  // Publish Message: Filtered cloud data
  stringstream filterMsg;
  filterMsg << "FILTERED POINTS: " << cloud_xyz->points.size();
  msg.data = filterMsg.str();
  msgPub.publish (msg);


  // EXTRACT KEYPOINTS
  PointCloud<PointXYZRGB>::Ptr current_keypoints (new PointCloud<PointXYZRGB>);
  search::KdTree<PointXYZRGB>::Ptr keypoint_tree (new search::KdTree<PointXYZRGB> ());

  // ISS 3D
  iss3d.setInputCloud(cloud_xyz);
  iss3d.setSearchMethod(keypoint_tree);
  iss3d.setSalientRadius (6 * cloud_resolution);
  iss3d.setNonMaxRadius (4 * cloud_resolution);
  iss3d.setThreshold21 (0.975);
  iss3d.setThreshold32 (0.975);
  iss3d.setMinNeighbors (5);
  iss3d.setNumberOfThreads (1);
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
  stringstream keypointsMsg;
  keypointsMsg << "KEYPOINTS FOUND: " << current_keypoints->points.size();
  msg.data = keypointsMsg.str();
  msgPub.publish (msg);

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
      stringstream noPrevious;
      noPrevious << "No Previous Cloud Data Was Found.........";
      msg.data = noPrevious.str();
      msgPub.publish (msg);

      // Set previous collections
      previous_keypoints->swap(*current_keypoints);
      previous_descriptors->swap(*current_descriptors);
      previousExists = true;

      // Publish Message: pervious cloud set
      stringstream setMsg;
      setMsg << "Previous Cloud Set.";
      msg.data = setMsg.str();
      msgPub.publish (msg);

    } else {

      // Publish Message: Previous cloud found
      stringstream foundPrevious;
      foundPrevious << "Previous Cloud Found!!!";
      msg.data = foundPrevious.str();
      msgPub.publish (msg);

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
      trans_est.estimateRigidTransformation (*current_keypoints, *previous_keypoints, *good_correspondences, transform);

      // Publish Message: Transformation
      stringstream resultsMsg;
      resultsMsg << "TRANSFORMATION: " << transform;
      msg.data = resultsMsg.str();
      msgPub.publish (msg);

      // Set previous collections
      previous_keypoints->swap(*current_keypoints);
      previous_descriptors->swap(*current_descriptors);
    }

    // Convert keypoint cloud ROS data type and publish
    PCLPointCloud2 currKeypoints;
    toPCLPointCloud2(*current_keypoints, currKeypoints);
    PointCloud2 output;
    fromPCL(currKeypoints, output);
    output.header.frame_id = "camera_link";
    keypointPub.publish(output);
  }

  // Publish Message: Done
  stringstream doneMsg;
  doneMsg << "------------------------------------------------";
  msg.data = doneMsg.str();
  msgPub.publish (msg);
}


int main (int argc, char** argv) {
  // Initialize ROS
  ros::init (argc, argv, "terrapin_pcl");
  NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  msgPub = nh.advertise<std_msgs::String> ("point_clouds/message_keypoints", 1);
  keypointPub = nh.advertise<PointCloud2> ("point_clouds/keypoints", 1);

  // Spin
  ros::spin ();
}
