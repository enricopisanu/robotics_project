#include "baseline_calculation_node.h"

#define ESTIMATED_GEAR_RATIO 38.34

void BaselineCalculationNode::OdometryCallback_(const robotics_hw1::MotorSpeed::ConstPtr &front_left_msg,
  const robotics_hw1::MotorSpeed::ConstPtr &front_right_msg,
  const robotics_hw1::MotorSpeed::ConstPtr &rear_left_msg,
  const robotics_hw1::MotorSpeed::ConstPtr &rear_right_msg,
  const nav_msgs::Odometry::ConstPtr &odom_msg)
{
  double average_left_rpm = (front_left_msg->rpm + rear_left_msg->rpm) / 2;
  double average_right_rpm = (front_right_msg->rpm + rear_right_msg->rpm) / 2;

  double v_left = rpmToSpeed(average_left_rpm);
  double v_right = rpmToSpeed(average_right_rpm);

  double v_x = (-v_left + v_right) / 2;

  double omega_odom = odom_msg->twist.twist.angular.z;
  double v_x_odom = odom_msg->twist.twist.linear.x;

  double mean_gear_ratio;

  if (v_x_odom != 0) {
    double gear_ratio = v_x / v_x_odom;
    gear_ratio_sum_ += gear_ratio;
    mean_gear_ratio = gear_ratio_sum_ / ++it_gear_ratio_;

    ROS_INFO("Gear ratio: %f", mean_gear_ratio);
  }

  if (abs(omega_odom) > 0.01 && v_left * v_right > 0) {
    double apparent_baseline = (v_left + v_right) / (ESTIMATED_GEAR_RATIO * omega_odom);
    apparent_baseline_sum_ += apparent_baseline;
    double mean_apparent_baseline = apparent_baseline_sum_ / ++it_apparent_baseline_;

    ROS_INFO("Apparent baseline: %f", mean_apparent_baseline);
  }
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "baseline_calculation_node");

  BaselineCalculationNode baseline_calculation;

  ros::spin();

  return 0;
}