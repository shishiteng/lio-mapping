#ifndef LOOP_CLOSURE_H
#define LOOP_CLOSURE_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/registration/gicp.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/filter.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/octree/octree_search.h>

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class LoopClosure
{
public:
  LoopClosure();
  ~LoopClosure();

  bool Initialize(const ros::NodeHandle &n);

  void HandleLoopClosures(bool bforce = true);

private:
  // Sensor callbacks.
  void PathCallback(const nav_msgs::Path::ConstPtr &msg);
  void PointCloudCallback(const PointCloud::ConstPtr &msg);

  gtsam::Pose3 RosToGtsam(geometry_msgs::Pose pose);
  gtsam::Pose3 EigenToGtsam(Eigen::Matrix4d transform);
  geometry_msgs::Pose EigenToRos(Eigen::Matrix4d transform);
  geometry_msgs::Pose GtsamToRos(gtsam::Pose3 pose);
  Eigen::Matrix4d GtsamToEigen(gtsam::Pose3 pose);
  Eigen::Matrix4d PosestampedToEigen(geometry_msgs::PoseStamped pose_stamped);

  void AngleFilter(const PointCloud::ConstPtr &points, PointCloud::Ptr points_filtered);
  void FiltPoints(const PointCloud::ConstPtr &points, PointCloud::Ptr points_filtered);
  bool FindPointCloud(double timestamp, PointCloud &cloud);

  bool PerformICP(const PointCloud::Ptr reference, const PointCloud::Ptr query, const Eigen::Matrix4d &pose0, gtsam::Pose3 &delta);

  void GenerateMap();
  void Generate3DofMap();

  void PublishResult(gtsam::Values final_estimate, std::vector<double> timestamp_buff);

  // The node's name.
  nav_msgs::Path path_;
  nav_msgs::Path path_loop_;
  nav_msgs::Path path_3dof_;

  // Subscribers.
  ros::Subscriber sub_path_;
  ros::Subscriber sub_points_;

  // Publishers
  ros::Publisher pub_path_;
  ros::Publisher pub_path_loop_;
  ros::Publisher pub_path_3dof_;
  ros::Publisher pub_octmap_;
  ros::Publisher pub_octmap_3dof_;

  double timeshift_;
  std::vector<PointCloud> points_buf_;
};

#endif
