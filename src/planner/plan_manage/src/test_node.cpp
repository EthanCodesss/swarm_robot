#include <chrono>
#include <fstream>
#include <iomanip>
#include <nav_msgs/Odometry.h>
#include <optimizer/poly_traj_utils.hpp>
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>
#include <sstream>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <traj_utils/PolyTraj.h>
#include <visualization_msgs/Marker.h>

ros::Publisher pos_cmd_pub;

quadrotor_msgs::PositionCommand cmd;
double pos_gain[3] = {0, 0, 0};
double vel_gain[3] = {0, 0, 0};

void cmdCallback(const ros::TimerEvent &e) {
  /* no publishing before receive traj_ */

  ros::Time time_now = ros::Time::now();
  double t_cur = (time_now - start_time_).toSec();

  Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()),
      acc(Eigen::Vector3d::Zero()), jerk(Eigen::Vector3d::Zero()), pos_f;
  std::pair<double, double> yaw_yawdot(0, 0);

  cmd.header.stamp = time_now;
  cmd.header.frame_id = "world";
  cmd.trajectory_flag =
      quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd.trajectory_id = 0;

  cmd.position.x = pos(0);
  cmd.position.y = pos(1);

  cmd.velocity.x = 0;
  cmd.velocity.y = 0;

  cmd.acceleration.x = 0;
  cmd.acceleration.y = 0;

  cmd.yaw = 0;
  cmd.yaw_dot = 0;

  pos_cmd_pub.publish(cmd);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "traj_server");
  // ros::NodeHandle node;
  ros::NodeHandle nh("~");
  // get drone num
  std::string name_drone = ros::this_node::getName();
  std::vector<std::string> v{explode(name_drone, '_')};
  // drone_id_ = std::stoi(v[1]);
  result_file.open(result_dir + v[1] + "_vaj.txt", std::ios::out);

  ros::Subscriber poly_traj_sub =
      nh.subscribe("planning/trajectory", 10, polyTrajCallback);
  ros::Subscriber reached_sub =
      nh.subscribe("planning/finish", 10, finishCallback);
  ros::Subscriber start_sub = nh.subscribe("planning/start", 10, startCallback);
  pos_cmd_pub =
      nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 50);

  ros::Timer cmd_timer = nh.createTimer(ros::Duration(0.01), cmdCallback);

  /* control parameter */
  cmd.kx[0] = pos_gain[0];
  cmd.kx[1] = pos_gain[1];
  cmd.kx[2] = pos_gain[2];

  cmd.kv[0] = vel_gain[0];
  cmd.kv[1] = vel_gain[1];
  cmd.kv[2] = vel_gain[2];

  nh.param("traj_server/time_forward", time_forward_, -1.0);
  last_yaw_ = 0.0;
  last_yaw_dot_ = 0.0;

  ros::Duration(1.0).sleep();

  ROS_WARN("[Traj server]: ready.");

  ros::spin();

  return 0;
}