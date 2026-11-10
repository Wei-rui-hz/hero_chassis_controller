#include "hero_chassis_controller/HeroChassisController.hpp"

#include <pluginlib/class_list_macros.h>

namespace hero_chassis_controller {

bool HeroChassisController::init(hardware_interface::EffortJointInterface* effort_joint_interface,
                                 ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) {
  const char* joint_names[4] = {"left_front_wheel_joint", "right_front_wheel_joint",
                                "left_back_wheel_joint", "right_back_wheel_joint"};
  for (int i = 0; i < 4; ++i) {
    joint_handle_[i] = effort_joint_interface->getHandle(joint_names[i]);
  }

  controller_nh.param("wheel_radius", wheel_radius_, 0.07625);
  controller_nh.param("half_length", half_length_, 0.2);
  controller_nh.param("half_width", half_width_, 0.2);
  controller_nh.param("proportional_gain", kp_, 1.0);

  cmd_vel_buffer_.writeFromNonRT(geometry_msgs::Twist());
  cmd_vel_sub_ = root_nh.subscribe("cmd_vel", 1, &HeroChassisController::cmdVelCallback, this);

  ROS_INFO_STREAM("[hero_chassis_controller] initialized: r=" << wheel_radius_
                  << " lx=" << half_length_ << " ly=" << half_width_ << " kp=" << kp_);
  return true;
}

void HeroChassisController::cmdVelCallback(const geometry_msgs::Twist& msg) {
  cmd_vel_buffer_.writeFromNonRT(msg);
}

void HeroChassisController::starting(const ros::Time& /*time*/) {
  for (int i = 0; i < 4; ++i) {
    desired_wheel_vel_[i] = 0.0;
    joint_handle_[i].setCommand(0.0);
  }
}

void HeroChassisController::update(const ros::Time& /*time*/, const ros::Duration& /*period*/) {
  geometry_msgs::Twist* cmd = cmd_vel_buffer_.readFromRT();

  const double vx = cmd ? cmd->linear.x : 0.0;
  const double vy = cmd ? cmd->linear.y : 0.0;
  const double wz = cmd ? cmd->angular.z : 0.0;
  const double l = half_length_ + half_width_;

  desired_wheel_vel_[0] = (vx - vy - l * wz) / wheel_radius_;
  desired_wheel_vel_[1] = (vx + vy + l * wz) / wheel_radius_;
  desired_wheel_vel_[2] = (vx + vy - l * wz) / wheel_radius_;
  desired_wheel_vel_[3] = (vx - vy + l * wz) / wheel_radius_;

  for (int i = 0; i < 4; ++i) {
    double torque = kp_ * (desired_wheel_vel_[i] - joint_handle_[i].getVelocity());
    if (torque > 5.0) torque = 5.0;
    if (torque < -5.0) torque = -5.0;
    joint_handle_[i].setCommand(torque);
  }
}

void HeroChassisController::stopping(const ros::Time& /*time*/) {
  for (int i = 0; i < 4; ++i) {
    joint_handle_[i].setCommand(0.0);
  }
}

}  // namespace hero_chassis_controller

PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController,
                       controller_interface::ControllerBase)