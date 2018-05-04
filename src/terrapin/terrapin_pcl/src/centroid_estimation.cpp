#include <sstream>
#include <iostream>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/keypoints/sift_keypoint.h>

using namespace std;
using namespace ros;
using namespace sensor_msgs;
using namespace pcl;
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
Publisher calculatedVelocityPub;
Publisher jsonMsgPub;

// Variables
const float filter_density = 0.01;
const int keypoint_threshold = 25;
const double cloud_resolution = 0.006;
// SIFT Parameters
const float min_scale = 0.01f;
const int n_octaves = 3;
const int n_scales_per_octave = 4;
const float min_contrast = 0.001f;

// PCL Operation Objects
VoxelGrid<PCLPointCloud2> voxel_grid;
SIFTKeypoint<PointXYZRGB, PointXYZRGB> sift;

// Previous Cloud Data
bool previousExists = false;
PointCloud<PointXYZRGB>::Ptr previous_keypoints (new PointCloud<PointXYZRGB>);
float x_previous;
float y_previous;
float z_previous;
int previous_cloud_num;
Time previous_timestamp;


// ********************** FUNCTIONS ***********************************

void cloud_cb (const PointCloud2ConstPtr& cloud_msg) {
  // Publish Message: analyzing cloud
  int current_cloud_num = cloud_msg->header.seq;
  cout << "Analyzing Cloud: " << current_cloud_num << endl;

  // Publish the original cloud
  originalCloudPub.publish(cloud_msg);

  // Get the current cloud's header data
  Time current_timestamp = cloud_msg->header.stamp;

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

  // Publish filtered point cloud
  PointCloud2 filteredOutput;
  fromPCL(cloud_filtered, filteredOutput);
  filteredOutput.header = cloud_msg->header;
  filteredCloudPub.publish(filteredOutput);


  // EXTRACT KEYPOINTS
  PointCloud<PointXYZRGB>::Ptr current_keypoints (new PointCloud<PointXYZRGB>);
  search::KdTree<PointXYZRGB>::Ptr keypoint_tree (new search::KdTree<PointXYZRGB> ());

  // SIFT Feature extractor
  sift.setInputCloud(cloud_xyz);
  sift.setSearchMethod(keypoint_tree);
  sift.setScales(min_scale, n_octaves, n_scales_per_octave);
  sift.setMinimumContrast(min_contrast);
  sift.compute(*current_keypoints);

  cout << "KEYPOINTS FOUND: " << current_keypoints->points.size() << endl;

  // Check if there are enough keypoints before continuing
  if (current_keypoints->points.size() >= keypoint_threshold) {

    // GET POINT CLOUD XYZ CENTROID
    float x_total = 0;
    float y_total = 0;
    float z_total = 0;
    for (int i = 0; i < current_keypoints->points.size(); i++) {
      x_total += current_keypoints->points[i].x;
      y_total += current_keypoints->points[i].y;
      z_total += current_keypoints->points[i].z;
    }
    float x_current = x_total / current_keypoints->points.size();
    float y_current = y_total / current_keypoints->points.size();
    float z_current = z_total / current_keypoints->points.size();

    // Check if previous cloud exists before continuing
    if (!previousExists) {
      // NO PREVIOUS POINT CLOUD
      cout << "No Previous Cloud Data Was Found..." << endl;

      // Set previous collections
      previous_keypoints->swap(*current_keypoints);
      x_previous = x_current;
      y_previous = y_current;
      z_previous = z_current;
      previous_cloud_num = current_cloud_num;
      previous_timestamp = current_timestamp;
      previousExists = true;

      // Publish Message: pervious cloud set
      cout << "Previous Cloud Set" << endl;
    } else {
      // PREVIOUS POINT CLOUD FOUND
      cout << "Previous Cloud Found!" << endl;

      // COMPARE CENTROIDS
      float x_diff = x_current - x_previous;
      float y_diff = y_current - y_previous;
      float z_diff = z_current - z_previous;
      float centroid_dist = sqrt(pow(x_diff,2) + pow(y_diff,2) + pow(z_diff,2));
      float time_diff = current_timestamp.toSec() - previous_timestamp.toSec();
      float velocity = centroid_dist / time_diff;

      cout << endl;
      cout << "X diff: " << x_diff << endl;
      cout << "Y diff: " << y_diff << endl;
      cout << "Z diff: " << y_diff << endl;
      cout << endl << "CENTROID DISPLACEMENT: " << centroid_dist << endl;
      cout << "TIME CHANGE: " << time_diff << endl;
      cout << endl << "VELOCITY: " << velocity << endl;


      // Publish Velocity data
      std_msgs::Float32 calculated_velocity;
      calculated_velocity.data = velocity;
      calculatedVelocityPub.publish(calculated_velocity);

      // Publish JSON data
      stringstream json;
      json << "{\"velocity\":" << velocity;
      json << ",\"displacement\":" << centroid_dist;
      json << ",\"deltaT\":" << time_diff;
      json << ",\"deltaX\":" << x_diff;
      json << ",\"deltaY\":" << y_diff;
      json << ",\"deltaZ\":" << z_diff;
      json << ",\"previousCloudNum\":" << previous_cloud_num;
      json << ",\"previousTimestamp\":" << previous_timestamp;
      json << ",\"previousNumKeypoints\":" << previous_keypoints->points.size();
      json << ",\"currentCloudNum\":" << current_cloud_num;
      json << ",\"currentTimestamp\":" << current_timestamp;
      json << ",\"currentNumKeypoints\":" << current_keypoints->points.size();
      json << "}";
      std_msgs::String rosJSON;
      rosJSON.data = json.str();
      jsonMsgPub.publish(rosJSON);

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
      x_previous = x_current;
      y_previous = y_current;
      z_previous = z_current;
      previous_cloud_num = current_cloud_num;
      previous_timestamp = current_timestamp;
    }
  }
  // Publish Message: Done
  cout << "------------------------------------------------" << endl << endl;
}


int main (int argc, char** argv) {
  // Initialize ROS
  ros::init (argc, argv, "terrapin_pcl");
  NodeHandle nh;

  // Create ROS Subscribers
  Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create ROS Publishers
  originalCloudPub = nh.advertise<PointCloud2> ("terrapin/point_clouds/original", 1);
  filteredCloudPub = nh.advertise<PointCloud2> ("terrapin/point_clouds/filtered", 1);
  currentKeypointsPub = nh.advertise<PointCloud2> ("terrapin/point_clouds/current_keypoints", 1);
  previousKeypointsPub = nh.advertise<PointCloud2> ("terrapin/point_clouds/previous_keypoints", 1);
  calculatedVelocityPub = nh.advertise<std_msgs::Float32> ("terrapin/calculated_velocity", 1);
  jsonMsgPub = nh.advertise<std_msgs::String> ("terrapin/json_message", 1);

  // Spin
  ros::spin ();
}
