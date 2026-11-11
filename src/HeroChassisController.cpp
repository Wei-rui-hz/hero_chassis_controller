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
  controller_nh.param("odom_publish_rate", odom_publish_rate_, 50.0);
  controller_nh.param<std::string>("velocity_mode", velocity_mode_, "chassis");
  controller_nh.param<std::string>("world_frame", world_frame_, "odom");
  if (velocity_mode_ != "chassis" && velocity_mode_ != "world") {
    ROS_WARN_STREAM("[hero_chassis_controller] unknown velocity_mode '" << velocity_mode_
                    << "', fallback to 'chassis'");
    velocity_mode_ = "chassis";
  }
  odom_pub_.init(root_nh, "odom", 4);
  reconfig_server_.reset(new dynamic_reconfigure::Server<HeroChassisControllerConfig>(controller_nh));
  reconfig_server_->setCallback(
      [this](HeroChassisControllerConfig& config, uint32_t level) { reconfigCallback(config, level); });

  cmd_vel_buffer_.writeFromNonRT(geometry_msgs::Twist());
  cmd_vel_sub_ = root_nh.subscribe("cmd_vel", 1, &HeroChassisController::cmdVelCallback, this);

  ROS_INFO_STREAM("[hero_chassis_controller] initialized: r=" << wheel_radius_
                  << " lx=" << half_length_ << " ly=" << half_width_
                  << " mode=" << velocity_mode_ << " (dynamic reconfigure ready)");
  return true;
}

void HeroChassisController::cmdVelCallback(const geometry_msgs::Twist& msg) {
  geometry_msgs::Twist cmd = msg;

  if (velocity_mode_ == "world") {
    tf::StampedTransform transform;
    try {
      // 取最新的 base_link 在 world_frame 下的位姿
      tf_listener_.lookupTransform(world_frame_, "base_link", ros::Time(0), transform);
    } catch (const tf::TransformException& ex) {
      ROS_WARN_THROTTLE(1.0, "[hero_chassis_controller] tf lookup failed: %s", ex.what());
      // 查不到变换就发零速度（启动瞬间 /odom 还没发出来时会走到这里）
      cmd_vel_buffer_.writeFromNonRT(geometry_msgs::Twist());
      return;
    }

    const double yaw = tf::getYaw(transform.getRotation());
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);

    // 世界系 -> 底盘系：绕 z 轴旋转 -yaw
    cmd.linear.x = c * msg.linear.x + s * msg.linear.y;
    cmd.linear.y = -s * msg.linear.x + c * msg.linear.y;
  }

  cmd_vel_buffer_.writeFromNonRT(cmd);
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
  odom_x_ = 0.0;
  odom_y_ = 0.0;
  odom_theta_ = 0.0;
  odom_pub_accum_ = 0.0;
}

void HeroChassisController::update(const ros::Time& time, const ros::Duration& period) {
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

    // ---- 正运动学：实际轮速 -> 车体速度 ----
  const double w1 = joint_handle_[0].getVelocity();  // LF
  const double w2 = joint_handle_[1].getVelocity();  // RF
  const double w3 = joint_handle_[2].getVelocity();  // LB
  const double w4 = joint_handle_[3].getVelocity();  // RB

  const double vx_act = wheel_radius_ * 0.25 * (w1 + w2 + w3 + w4);
  const double vy_act = wheel_radius_ * 0.25 * (-w1 + w2 + w3 - w4);
  const double wz_act = wheel_radius_ / (4.0 * l) * (-w1 + w2 - w3 + w4);

  // ---- 积分成位姿 ----
  const double dt = period.toSec();
  const double cos_t = std::cos(odom_theta_);
  const double sin_t = std::sin(odom_theta_);
  odom_x_ += (vx_act * cos_t - vy_act * sin_t) * dt;
  odom_y_ += (vx_act * sin_t + vy_act * cos_t) * dt;
  odom_theta_ += wz_act * dt;

  // ---- 按 publish_rate 节流发布 ----
  odom_pub_accum_ += dt;
  if (odom_pub_accum_ >= 1.0 / odom_publish_rate_) {
    odom_pub_accum_ = 0.0;

    const geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(odom_theta_);

    if (odom_pub_.trylock()) {
      odom_pub_.msg_.header.stamp = time;
      odom_pub_.msg_.header.frame_id = "odom";
      odom_pub_.msg_.child_frame_id = "base_link";

      odom_pub_.msg_.pose.pose.position.x = odom_x_;
      odom_pub_.msg_.pose.pose.position.y = odom_y_;
      odom_pub_.msg_.pose.pose.position.z = 0.0;
      odom_pub_.msg_.pose.pose.orientation = q;

      odom_pub_.msg_.twist.twist.linear.x = vx_act;
      odom_pub_.msg_.twist.twist.linear.y = vy_act;
      odom_pub_.msg_.twist.twist.angular.z = wz_act;
      odom_pub_.unlockAndPublish();
    }

    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp = time;
    tf_msg.header.frame_id = "odom";
    tf_msg.child_frame_id = "base_link";
    tf_msg.transform.translation.x = odom_x_;
    tf_msg.transform.translation.y = odom_y_;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation = q;
    tf_broadcaster_.sendTransform(tf_msg);
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