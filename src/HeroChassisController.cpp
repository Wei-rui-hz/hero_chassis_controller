#include "hero_chassis_controller/HeroChassisController.hpp"

#include <pluginlib/class_list_macros.h>

namespace hero_chassis_controller {

bool HeroChassisController::init(hardware_interface::EffortJointInterface* effort_joint_interface,
                                 ros::NodeHandle& /*root_nh*/,
                                 ros::NodeHandle& /*controller_nh*/) {
  const char* joint_names[4] = {"left_front_wheel_joint", "right_front_wheel_joint",
                                "left_back_wheel_joint", "right_back_wheel_joint"};
  for (int i = 0; i < 4; ++i) {
    joint_handle_[i] = effort_joint_interface->getHandle(joint_names[i]);
  }
  return true;
}

void HeroChassisController::starting(const ros::Time& /*time*/) {
  for (auto& j : joint_handle_) {
    j.setCommand(0.0);
  }
}

void HeroChassisController::update(const ros::Time& /*time*/, const ros::Duration& /*period*/) {
  for (auto& j : joint_handle_) {
    j.setCommand(0.0);
  }
}

void HeroChassisController::stopping(const ros::Time& /*time*/) {
  for (auto& j : joint_handle_) {
    j.setCommand(0.0);
  }
}

}  // namespace hero_chassis_controller

PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController,
                       controller_interface::ControllerBase)
