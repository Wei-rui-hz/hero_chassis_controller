#pragma once

#include <controller_interface/controller.h>
#include <geometry_msgs/Twist.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_buffer.h>
#include <ros/ros.h>

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

  hardware_interface::JointHandle joint_handle_[4];  // LF RF LB RB
  double desired_wheel_vel_[4]{};

  double wheel_radius_{0.07625};
  double half_length_{0.2};
  double half_width_{0.2};
  double kp_{1.0};

  realtime_tools::RealtimeBuffer<geometry_msgs::Twist> cmd_vel_buffer_;
  ros::Subscriber cmd_vel_sub_;
};

}  // namespace hero_chassis_controller