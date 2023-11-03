#include <math.h>

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <geometry_msgs/TwistStamped.h>

#include "robotics_hw1/MotorSpeed.h"

#define RADIUS 0.1575
#define GEAR_RATIO 38.34
#define APPARENT_BASELINE 1.03

double rpmToSpeed(double rpm){
    return rpm * 2 * M_PI * RADIUS / (60 * GEAR_RATIO);
}

void computeSpeed(const robotics_hw1::MotorSpeedConstPtr& front_left, 
              const robotics_hw1::MotorSpeedConstPtr& front_right,
              const robotics_hw1::MotorSpeedConstPtr& rear_left, 
              const robotics_hw1::MotorSpeedConstPtr& rear_right,
              const ros::Publisher speed_pub) {

    double average_left_rpm = (front_left->rpm + rear_left->rpm)/2;
    double average_right_rpm = (front_right->rpm + rear_right->rpm)/2;

    double v_left = rpmToSpeed(average_left_rpm);
    double v_right = rpmToSpeed(average_right_rpm);

    double linear_speed = (-v_left + v_right)/2;
    double angular_speed = (v_left + v_right)/APPARENT_BASELINE;

    geometry_msgs::TwistStamped speed_msg;
    speed_msg.header.stamp = front_left->header.stamp;
    speed_msg.twist.linear.x = linear_speed;
    speed_msg.twist.angular.z = angular_speed;

    speed_pub.publish(speed_msg);

}

int main(int argc, char** argv) {

    ros::init(argc, argv, "speed_publisher_node");

    ros::NodeHandle nh;

    ros::Publisher speed_pub = nh.advertise<geometry_msgs::TwistStamped>("/speed_odom", 1000);

    typedef message_filters::Subscriber<robotics_hw1::MotorSpeed> MessageFilterSubscriber;
    MessageFilterSubscriber fl_sub(nh, "motor_speed_fl", 1);
    MessageFilterSubscriber fr_sub(nh, "motor_speed_fr", 1);
    MessageFilterSubscriber rl_sub(nh, "motor_speed_rl", 1);
    MessageFilterSubscriber rr_sub(nh, "motor_speed_rr", 1);
    
    message_filters::TimeSynchronizer<robotics_hw1::MotorSpeed, 
                                        robotics_hw1::MotorSpeed,
                                        robotics_hw1::MotorSpeed, 
                                        robotics_hw1::MotorSpeed> sync(fl_sub, fr_sub, rl_sub, rr_sub, 10);
    sync.registerCallback(boost::bind(&computeSpeed, _1, _2, _3, _4, speed_pub));

    ros::spin();

    return 0;
    
}