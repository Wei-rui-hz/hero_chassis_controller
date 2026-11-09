# hero_chassis_controller（Qiuweiling）

## Overview

麦克纳姆轮英雄机器人底盘控制器，基于 ros_control 实现。接收 `geometry_msgs/Twist`
速度指令，通过逆运动学解算四个麦轮的目标转速，并用力矩闭环控制轮毂电机。

**Keywords:** robomaster, mecanum, chassis, ros_control

## License

The source code is released under a [BSD 3-Clause license](LICENSE).

**Author: Qiuweiling<br />
Maintainer: Qiuweiling, 2465308638@qq.com**

## Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) Noetic
- [Eigen](http://eigen.tuxfamily.org) 线性代数库
- ros_control

## Building

使用 CLion 构建：

1. 用 CLion 打开 catkin 工作空间，CMake options 设置为
   `-DCMAKE_BUILD_TYPE=Debug -DCATKIN_DEVEL_PREFIX=<ws>/devel -DCMAKE_INSTALL_PREFIX=<ws>/install -DCMAKE_PREFIX_PATH=/opt/ros/noetic`
2. `文件 > 重新加载 CMake 项目`，然后 `构建 > 重新构建项目`
3. 产物位于 `<ws>/devel/lib/libhero_chassis_controller_core.so`

## Usage

待补充。

## Launch files

待补充。

## Config files

待补充。