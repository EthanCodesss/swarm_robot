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
  if (!receive_traj_)
    return;

  ros::Time time_now = ros::Time::now();
  double t_cur = (time_now - start_time_).toSec();

  Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()),
      acc(Eigen::Vector3d::Zero()), jerk(Eigen::Vector3d::Zero()), pos_f;
  std::pair<double, double> yaw_yawdot(0, 0);

  static ros::Time time_last = ros::Time::now();
  if (flag == false) {
    flag = true;
  } else {
    cnt++;
    acc2_inter +=
        last_acc.norm() * last_acc.norm() * (time_now - time_last).toSec();
    jerk2_inter +=
        last_jerk.norm() * last_jerk.norm() * (time_now - time_last).toSec();
  }
  if (t_cur < traj_duration_ && t_cur >= 0.0) {
    pos = traj_->getPos(t_cur);
    vel = traj_->getVel(t_cur);
    acc = traj_->getAcc(t_cur);
    jerk = traj_->getJer(t_cur);

    /*** calculate yaw ***/
    yaw_yawdot = calculate_yaw(t_cur, pos, time_now, time_last);
    /*** calculate yaw ***/

    double tf = std::min(traj_duration_, t_cur + 2.0);
    pos_f = traj_->getPos(tf);
  } else if (t_cur >= traj_duration_) {
    /* hover when finish traj_ */
    pos = traj_->getPos(traj_duration_);
    vel.setZero();
    acc.setZero();

    yaw_yawdot.first = last_yaw_;
    yaw_yawdot.second = 0;

    pos_f = pos;
  } else {
    // std::cout << "[Traj server]: invalid time." << std::endl;
  }
  time_last = time_now;
  time_vec_.push_back((ros::Time::now() - global_start_time).toSec());
  pos_vec_.push_back(pos);
  vel_vec_.push_back(vel);
  acc_vec_.push_back(acc);
  jerk_vec_.push_back(jerk);
  last_vel = vel;
  last_acc = acc;
  last_jerk = jerk;

  cmd.header.stamp = time_now;
  cmd.header.frame_id = "world";
  cmd.trajectory_flag =
      quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd.trajectory_id = traj_id_;

  cmd.position.x = pos(0);
  cmd.position.y = pos(1);
  cmd.position.z = pos(2);

  cmd.velocity.x = vel(0);
  cmd.velocity.y = vel(1);
  cmd.velocity.z = vel(2);

  cmd.acceleration.x = acc(0);
  cmd.acceleration.y = acc(1);
  cmd.acceleration.z = acc(2);

  cmd.yaw = yaw_yawdot.first;
  cmd.yaw_dot = yaw_yawdot.second;

  last_yaw_ = cmd.yaw;

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