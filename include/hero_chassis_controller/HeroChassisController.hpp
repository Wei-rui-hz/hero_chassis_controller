#pragma once

#include <controller_interface/controller.h>
#include <geometry_msgs/Twist.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_buffer.h>
#include <ros/ros.h>
#include <control_toolbox/pid.h>
#include <dynamic_reconfigure/server.h>
#include <hero_chassis_controller/HeroChassisControllerConfig.h>
#include <cmath>
#include <nav_msgs/Odometry.h>
#include <realtime_tools/realtime_publisher.h>
#include <tf/transform_broadcaster.h>

namespace hero_chassis_controller {

class HeroChassisController
    : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
 public:
  bool init(hardware_interface::EffortJointInterface* effort_joint_interface,
            ros::NodeHandle& root_nh,
            ros::NodeHandle& controller_nh) override;

  void starting(const ros::Time& time) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void stopping(const ros::Time& time) override;

 private:
  void cmdVelCallback(const geometry_msgs::Twist& msg);
  void reconfigCallback(HeroChassisControllerConfig& config, uint32_t level);
  boost::shared_ptr<dynamic_reconfigure::Server<HeroChassisControllerConfig>> reconfig_server_;

  hardware_interface::JointHandle joint_handle_[4];  // LF RF LB RB
  double desired_wheel_vel_[4]{};  // LF RF LB RB

  control_toolbox::Pid pid_[4];  // 一个轮子一个，各自持有积分状态

  double wheel_radius_{0.07625};
  double half_length_{0.2};
  double half_width_{0.2};

  realtime_tools::RealtimeBuffer<geometry_msgs::Twist> cmd_vel_buffer_;
  ros::Subscriber cmd_vel_sub_;
    // ---- 里程计 ----
  double odom_x_{0.0};
  double odom_y_{0.0};
  double odom_theta_{0.0};
  double odom_publish_rate_{50.0};
  double odom_pub_accum_{0.0};  // 发布节流累加器
  realtime_tools::RealtimePublisher<nav_msgs::Odometry> odom_pub_;
  tf::TransformBroadcaster tf_broadcaster_;
};

}  // namespace hero_chassis_controller