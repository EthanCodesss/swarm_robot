#include "quadrotor_msgs/PositionCommand.h"
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <iostream>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <random>
#include <ros/ros.h>
#define PI 3.1415926
#define DTR PI / 180.0 // Degree to Rad
#define RTD 180.0 / PI // Rad to Degree

ros::Subscriber _cmd_sub;
ros::Publisher _odom_pub;

quadrotor_msgs::PositionCommand _cmd;
double _init_x, _init_y;

bool rcv_cmd = false;
// fstream file
void rcvPosCmdCallBack(const quadrotor_msgs::PositionCommand cmd) {
  rcv_cmd = true;
  _cmd = cmd;
}

void pubOdom() {
  nav_msgs::Odometry odom;
  odom.header.stamp = ros::Time::now();
  odom.header.frame_id = "world";

  if (rcv_cmd) {
    odom.pose.pose.position.x = _cmd.position.x;
    odom.pose.pose.position.y = _cmd.position.y;

    // Eigen::Vector3d alpha =
    //     Eigen::Vector3d(_cmd.acceleration.x, _cmd.acceleration.y,
    //                     _cmd.acceleration.z) +
    //     9.8 * Eigen::Vector3d(0, 0, 1);
    // Eigen::Vector3d xC(cos(_cmd.yaw), sin(_cmd.yaw), 0);
    // Eigen::Vector3d yC(-sin(_cmd.yaw), cos(_cmd.yaw), 0);
    // Eigen::Vector3d xB = (yC.cross(alpha)).normalized();
    // Eigen::Vector3d yB = (alpha.cross(xB)).normalized();
    // Eigen::Vector3d zB = xB.cross(yB);
    // Eigen::Matrix3d R;
    // R.col(0) = xB;
    // R.col(1) = yB;
    // R.col(2) = zB;
    // Eigen::Quaterniond q(R);
    odom.pose.pose.orientation.w = 1;
    odom.pose.pose.orientation.x = 0;
    odom.pose.pose.orientation.y = 0;
    odom.pose.pose.orientation.z = 0;

    odom.twist.twist.linear.x = _cmd.velocity.x;
    odom.twist.twist.linear.y = _cmd.velocity.y;

    odom.twist.twist.angular.x = _cmd.acceleration.x;
    odom.twist.twist.angular.y = _cmd.acceleration.y;

  } else {
    odom.pose.pose.position.x = _init_x;
    odom.pose.pose.position.y = _init_y;

    odom.pose.pose.orientation.w = 1;
    odom.pose.pose.orientation.x = 0;
    odom.pose.pose.orientation.y = 0;
    odom.pose.pose.orientation.z = 0;

    odom.twist.twist.linear.x = 0.0;
    odom.twist.twist.linear.y = 0.0;

    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
  }

  _odom_pub.publish(odom);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "odom_generator");
  ros::NodeHandle nh("~");

  nh.param("init_x", _init_x, 0.0);
  nh.param("init_y", _init_y, 0.0);
  nh.param("init_z", _init_z, 0.0);

  _cmd_sub = nh.subscribe("command", 1, rcvPosCmdCallBack);
  _odom_pub = nh.advertise<nav_msgs::Odometry>("odometry", 1);

  ros::Rate rate(100);
  bool status = ros::ok();
  while (status) {
    pubOdom();
    ros::spinOnce();
    status = ros::ok();
    rate.sleep();
  }

  return 0;
}