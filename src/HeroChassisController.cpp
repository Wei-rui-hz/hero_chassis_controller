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
  reconfig_server_.reset(new dynamic_reconfigure::Server<HeroChassisControllerConfig>(controller_nh));
  reconfig_server_->setCallback(
      [this](HeroChassisControllerConfig& config, uint32_t level) { reconfigCallback(config, level); });

  cmd_vel_buffer_.writeFromNonRT(geometry_msgs::Twist());
  cmd_vel_sub_ = root_nh.subscribe("cmd_vel", 1, &HeroChassisController::cmdVelCallback, this);

  ROS_INFO_STREAM("[hero_chassis_controller] initialized: r=" << wheel_radius_
                  << " lx=" << half_length_ << " ly=" << half_width_ << " (dynamic reconfigure ready)");
  return true;
}

void HeroChassisController::cmdVelCallback(const geometry_msgs::Twist& msg) {
  cmd_vel_buffer_.writeFromNonRT(msg);
}

void HeroChassisController::reconfigCallback(HeroChassisControllerConfig& config, uint32_t /*level*/) {
  pid_[0].setGains(config.lf_p, config.lf_i, config.lf_d, config.lf_i_clamp_max, config.lf_i_clamp_min);
  pid_[1].setGains(config.rf_p, config.rf_i, config.rf_d, config.rf_i_clamp_max, config.rf_i_clamp_min);
  pid_[2].setGains(config.lb_p, config.lb_i, config.lb_d, config.lb_i_clamp_max, config.lb_i_clamp_min);
  pid_[3].setGains(config.rb_p, config.rb_i, config.rb_d, config.rb_i_clamp_max, config.rb_i_clamp_min);
}

void HeroChassisController::starting(const ros::Time& /*time*/) {
  for (int i = 0; i < 4; ++i) {
    desired_wheel_vel_[i] = 0.0;
    pid_[i].reset();
    joint_handle_[i].setCommand(0.0);
  }
}

void HeroChassisController::update(const ros::Time& /*time*/, const ros::Duration& period) {
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
    const double error = desired_wheel_vel_[i] - joint_handle_[i].getVelocity();
    double torque = pid_[i].computeCommand(error, period);
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