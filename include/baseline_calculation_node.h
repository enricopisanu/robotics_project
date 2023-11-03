#include <stddef.h>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "robotics_hw1/MotorSpeed.h"
#include <nav_msgs/Odometry.h>

class BaselineCalculationNode {

  public:
    BaselineCalculationNode(){
      fl_.subscribe(nh_, "motor_speed_fl", 1);
      fr_.subscribe(nh_, "motor_speed_fr", 1);
      rl_.subscribe(nh_, "motor_speed_rl", 1);
      rr_.subscribe(nh_, "motor_speed_rr", 1);
      odom_.subscribe(nh_, "scout_odom", 1);
      sync_.reset(new Sync(MySyncPolicy(1), fl_, fr_, rl_, rr_, odom_));
      sync_->registerCallback(boost::bind(&BaselineCalculationNode::OdometryCallback_, this, _1, _2, _3, _4, _5));

    };
  

  private:
    void LaunchSubcribers_();
    void OdometryCallback_(const robotics_hw1::MotorSpeed::ConstPtr& front_left, 
              const robotics_hw1::MotorSpeed::ConstPtr& front_right,
              const robotics_hw1::MotorSpeed::ConstPtr& rear_left, 
              const robotics_hw1::MotorSpeed::ConstPtr& rear_right,
              const nav_msgs::Odometry::ConstPtr& odom_msg);
    double rpmToSpeed(double rpm){
      return rpm * 2 * M_PI * radius_ / 60;
}
  private:
    ros::NodeHandle nh_;
    typedef message_filters::sync_policies::ApproximateTime<robotics_hw1::MotorSpeed, robotics_hw1::MotorSpeed, robotics_hw1::MotorSpeed, robotics_hw1::MotorSpeed, nav_msgs::Odometry> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    typedef message_filters::Subscriber<robotics_hw1::MotorSpeed> MessageFilterSubscriber;
    MessageFilterSubscriber fl_;
    MessageFilterSubscriber fr_;
    MessageFilterSubscriber rl_;
    MessageFilterSubscriber rr_;
    message_filters::Subscriber<nav_msgs::Odometry> odom_;
    
    boost::shared_ptr<Sync> sync_;

    double gear_ratio_sum_ = 0;
    size_t it_gear_ratio_ = 0;

    double apparent_baseline_sum_ = 0;
    size_t it_apparent_baseline_ = 0;

    const double radius_ = 0.1575;
};