#pragma once

#include <eigen3/Eigen/Dense>
#include <yaml-cpp/yaml.h>

// ROS Libraries
#include <ros/console.h>
#include <ros/package.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

// Project Packages
#include "utils/math.h"

// Messages
#include "robotics_hw1/OdometryCustom.h"
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

// Services
#include "robotics_hw1/SetOdom.h"
#include "std_srvs/Empty.h"

// Dynamic Reconfigure
#include "dynamic_reconfigure/server.h"
#include "robotics_hw1/IntegrationConfig.h"

#include <filesystem>
#include <fstream>
#include <gflags/gflags.h>

DEFINE_string(bag_filename, "", "Set bag filename for initial values");

class OdometryNode
{
public:
  // CONSTRUCTORS AND DESTRUCTORS
  OdometryNode(const std::string &bag_filename);
  ~OdometryNode(){};

private:
  // ATTRIBUTES
  ros::NodeHandle nh_;
  std::optional<geometry_msgs::TwistStamped> last_odom_msg_;// Last received odometry message
  bool publish_tf_ = true;

  // SUBSCRIBERS
  ros::Subscriber odom_sub_;

  // PUBLISHERS
  ros::Publisher odom_pub_;
  ros::Publisher odom_custom_pub_;
  nav_msgs::Odometry odom_pub_msg_;
  robotics_hw1::OdometryCustom odom_custom_pub_msg_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  geometry_msgs::TransformStamped tf_msg_;

  // SERVICES
  ros::ServiceServer set_odom_to_pose_srv_;
  ros::ServiceServer set_odom_to_zero_srv_;

  // DYNAMIC RECONFIGURE
  bool first_time_reconfigure_ = true;
  dynamic_reconfigure::Server<robotics_hw1::IntegrationConfig> dynamic_server_;
  dynamic_reconfigure::Server<robotics_hw1::IntegrationConfig>::CallbackType callback_dynamic_reconfigure_;

  void LoadInitStateYAML_(const std::string &bag_filename);
  void LoadInitState_();
  bool CheckParameterExistence_(const std::string &param_name, double &parameter);

  void LaunchSubscribers_();
  void LaunchPublishers_();
  void LaunchServices_();
  void LaunchDynamicReconfigure_();
  void ChooseIntegrationMethod_(robotics_hw1::IntegrationConfig &config, uint32_t level);

  bool SetOdometryToPose_(robotics_hw1::SetOdom::Request &req, robotics_hw1::SetOdom::Response &res);
  bool SetOdometryToZero_(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

  void BroadcastTf_();

  // CALLBACKS
  void OdomCallback_(const geometry_msgs::TwistStamped::ConstPtr &msg);

  ros::Time last_time_;
  std::optional<ros::Time> last_tf_time_;

  struct PoseType
  {
    double x;
    double y;
    double theta;
    double vx;
    double omega;
    bool use_runge_kutta;
  };
  PoseType pose_;


  double yawFromQuaternion(Eigen::Quaterniond q)
  {
    double siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());

    return std::atan2(siny_cosp, cosy_cosp);
  }
};