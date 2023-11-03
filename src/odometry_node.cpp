#include "odometry_node.h"

OdometryNode::OdometryNode(const std::string& bag_filename){
  odom_pub_msg_.header.frame_id = "odom";
  odom_pub_msg_.child_frame_id = "base_link";

  odom_custom_pub_msg_.odom.header.frame_id = "odom";
  odom_custom_pub_msg_.odom.child_frame_id = "base_link";

  LaunchDynamicReconfigure_();
  // LoadInitState_();
  LoadInitStateYAML_(bag_filename);
  LaunchSubscribers_();
  LaunchPublishers_();
  LaunchServices_();
}

void OdometryNode::LoadInitStateYAML_(const std::string& bag_filename){

  auto current_dir = std::filesystem::current_path();
  YAML::Node config;
  try {
    config = YAML::LoadFile(current_dir.string() +
              "/../robotics_project/src/robotics_hw1/cfg/initial_pose.yaml");
  } catch (const YAML::ParserException& ex) {
    std::cout << ex.what() << std::endl;
  } catch (const YAML::BadFile& ex) {
    std::cout << ex.what() << std::endl;
    }
  
  if(config["initial_pose"]){
    if(config["initial_pose"][bag_filename]["position"] && config["initial_pose"][bag_filename]["position"]["x"] && config["initial_pose"][bag_filename]["position"]["y"] ){
      pose_.x = config["initial_pose"][bag_filename]["position"]["x"].as<double>();
      pose_.y = config["initial_pose"][bag_filename]["position"]["y"].as<double>();
    }
    else{
      ROS_WARN_STREAM("Cannot find position field or x and y fields in file. Setting default value.");
      pose_.x = 0;
      pose_.y = 0;
    }
    if(config["initial_pose"][bag_filename]["orientation"] && config["initial_pose"][bag_filename]["orientation"]["x"] &&
      config["initial_pose"][bag_filename]["orientation"]["y"] && config["initial_pose"][bag_filename]["orientation"]["z"]
      && config["initial_pose"][bag_filename]["orientation"]["w"]){
      Eigen::Quaterniond quat;
      quat.x()       = config["initial_pose"][bag_filename]["orientation"]["x"].as<double>();
      quat.y()       = config["initial_pose"][bag_filename]["orientation"]["y"].as<double>();
      quat.z()       = config["initial_pose"][bag_filename]["orientation"]["z"].as<double>();
      quat.w()       = config["initial_pose"][bag_filename]["orientation"]["w"].as<double>();

      pose_.theta = yawFromQuaternion(quat);
    }
    else{
      ROS_WARN_STREAM("Cannot find orientation field or quaternion component's fields in file. Setting default value.");
      pose_.theta = 0;
    }
  }
  else{
    ROS_WARN_STREAM("Cannot find initial_pose field in file. Setting default values.");
    pose_.x = 0;
    pose_.y = 0;
    pose_.theta = 0;
  }
  
  ROS_INFO("Initial pose set at (x, y, theta) = (%2f, %2f, %2f).", pose_.x, pose_.y, pose_.omega );
};

void OdometryNode::LoadInitState_(){
  if(!this->CheckParameterExistence_("x0", pose_.x)) 
    pose_.x = 0.0;
  if(!this->CheckParameterExistence_("y0", pose_.y)) 
    pose_.y = 0.0;
  if(!this->CheckParameterExistence_("theta0", pose_.theta)){
      pose_.theta = 0.0;
  }
  ROS_INFO("Initial pose set at (x, y, theta) = (%2f, %2f, %2f).", pose_.x, pose_.y, pose_.omega );
}

bool OdometryNode::CheckParameterExistence_(const std::string &param_name, double &parameter){
  if (!ros::param::has(param_name)) {
      ROS_WARN_STREAM("Param " + param_name + " does not exists in rosparam server. Setting default value.");
      return false;
    }
  ros::param::get(param_name, parameter);
  return true;  
}

void OdometryNode::LaunchSubscribers_(){
  odom_sub_ = this->nh_.subscribe("/speed_odom", 1000, &OdometryNode::OdomCallback_, this);
}

void OdometryNode::LaunchPublishers_(){
  odom_pub_ = this->nh_.advertise<nav_msgs::Odometry>("/odometry", 1000); 
  odom_custom_pub_ = this->nh_.advertise<robotics_hw1::OdometryCustom>("/odometry_custom", 1000); 
}


void OdometryNode::LaunchServices_(){
  set_odom_to_zero_srv_ = nh_.advertiseService<std_srvs::Empty::Request,
                                                          std_srvs::Empty::Response>
                                                                ("set_odometry_to_zero", boost::bind(&OdometryNode::SetOdometryToZero_, this, _1, _2));
  set_odom_to_pose_srv_ = nh_.advertiseService<robotics_hw1::SetOdom::Request,
                                                                robotics_hw1::SetOdom::Response>
                                                          ("set_odometry_to_pose", boost::bind(&OdometryNode::SetOdometryToPose_, this, _1, _2));
  
}

bool OdometryNode::SetOdometryToPose_(robotics_hw1::SetOdom::Request &req,
                robotics_hw1::SetOdom::Response &res){
  pose_.x = req.x;
  pose_.y = req.y;
  pose_.theta = req.theta;
    
  ROS_INFO("Odometry setted at (x, y, theta) = (%f, %f, %f).", pose_.x, pose_.y, pose_.theta);
  return true;
}

bool OdometryNode::SetOdometryToZero_(std_srvs::Empty::Request &req,
                std_srvs::Empty::Response &res){
  pose_.x = 0.0;
  pose_.y = 0.0;
  pose_.theta = 0.0;
  
  ROS_INFO("Odometry setted at (x, y, theta) = (0, 0, 0).");
  return true;
}

void OdometryNode::OdomCallback_(const geometry_msgs::TwistStamped::ConstPtr &msg){
  if (!last_odom_msg_.has_value()) {
    last_odom_msg_ = std::make_optional<geometry_msgs::TwistStamped>(*msg);
    return;
  }

  pose_.vx = msg->twist.linear.x;
  pose_.omega = msg->twist.angular.z;
  
  double dt = msg->header.stamp.toSec() - last_odom_msg_->header.stamp.toSec();

  double argument_sincos;
  argument_sincos = pose_.use_runge_kutta? pose_.theta + pose_.omega*dt/2 : pose_.theta;

  pose_.x     += pose_.vx * dt * cos( argument_sincos );
  pose_.y     += pose_.vx * dt * sin( argument_sincos );
  pose_.theta += pose_.omega* dt;

  ROS_DEBUG_STREAM("Delta time: " << dt );
  ROS_DEBUG_STREAM("Speed: " << pose_.vx << " " << pose_.omega );
  ROS_DEBUG_STREAM("Position: " << pose_.x << ", " << pose_.y);
  
  BroadcastTf_();

  tf2::Quaternion tf_quat;
  tf_quat.setRPY(0.0, 0.0, pose_.theta);
  geometry_msgs::Quaternion quat;
  tf2::convert(tf_quat, quat);

  odom_pub_msg_.header.stamp = msg->header.stamp;
  odom_pub_msg_.pose.pose.position.x = pose_.x;
  odom_pub_msg_.pose.pose.position.y = pose_.y;
  odom_pub_msg_.pose.pose.orientation = quat;
  odom_pub_msg_.twist.twist.linear.x = pose_.vx;
  odom_pub_msg_.twist.twist.angular.z = pose_.omega;

  odom_custom_pub_msg_.odom.header.stamp = msg->header.stamp;
  odom_custom_pub_msg_.odom.pose.pose.position.x = pose_.x;
  odom_custom_pub_msg_.odom.pose.pose.position.y = pose_.y;
  odom_custom_pub_msg_.odom.pose.pose.orientation = quat;
  odom_custom_pub_msg_.odom.twist.twist.linear.x = pose_.vx;
  odom_custom_pub_msg_.odom.twist.twist.angular.z = pose_.omega;
  odom_custom_pub_msg_.integration_method.data = pose_.use_runge_kutta? "Runge-Kutta" : "Euler";

  odom_pub_.publish(odom_pub_msg_);
  odom_custom_pub_.publish(odom_custom_pub_msg_);

  last_odom_msg_ = *msg;
}

void OdometryNode::LaunchDynamicReconfigure_(){
  callback_dynamic_reconfigure_ = boost::bind(&OdometryNode::ChooseIntegrationMethod_, this, _1, _2);
  dynamic_server_.setCallback(callback_dynamic_reconfigure_);
}

void OdometryNode::ChooseIntegrationMethod_(robotics_hw1::IntegrationConfig &config,
                    uint32_t level){
  std::string chosen_type = config.integration? "Runge-Kutta" : "Euler"; 
  pose_.use_runge_kutta = config.integration;

  if(first_time_reconfigure_){
    ROS_INFO("Initial integration method: %s", chosen_type.c_str());
    first_time_reconfigure_ = false;
    return;
  } 

  ROS_INFO("[RECONFIGURE REQUEST] Change integration method to %s", chosen_type.c_str());

}


void OdometryNode::BroadcastTf_(){
  if(publish_tf_){
    if (last_tf_time_.has_value() && ros::Time::now() == last_tf_time_) {
      return;
    }

    tf2::Stamped<tf2::Transform> tf;
    tf2::Quaternion quat;
    quat.setRPY(0.0,0.0, pose_.theta);
    quat.normalize();
    tf.getOrigin().setValue(pose_.x, pose_.y, 0.0);
    tf.setRotation(quat);

    // Convert and publish
    tf_msg_ = tf2::toMsg(tf);
    tf_msg_.header.frame_id = "odom";
    tf_msg_.child_frame_id = "base_link";
    tf_msg_.header.stamp = ros::Time::now();
    tf_broadcaster_.sendTransform(tf_msg_);
    last_tf_time_ = std::make_optional<ros::Time>(tf_msg_.header.stamp);
  }
}

// MAIN
int main(int argc, char **argv) {
  ros::init(argc, argv, "odometry_node");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  if (FLAGS_bag_filename.empty()) {
    ROS_FATAL_STREAM("Please provide bag filename");
  }
  OdometryNode odometryNode(FLAGS_bag_filename);
  ros::spin();
  return 0;
}