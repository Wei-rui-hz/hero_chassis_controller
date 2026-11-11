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

详见下方[使用说明](#使用说明)。

## Launch files

- `launch/load_controller.launch`：加载 yaml 参数，并通过 `controller_manager/spawner` 启动控制器。

## Config files

- `config/hero_chassis_controller.yaml`：底盘几何参数（`wheel_radius` / `half_length` / `half_width`）、速度模式（`velocity_mode`）、里程计发布频率（`odom_publish_rate`）、四个轮子的 PID 参数。

## 使用说明

### 启动

```bash
roslaunch rm_description_for_task hero.launch
roslaunch hero_chassis_controller load_controller.launch
```

### 键盘操控

```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

采用类似手柄的按键位置布局：`i` 前进、`,` 后退、`j`/`l` 左转右转、`u`/`o`/`m`/`.` 斜向、`q`/`z` 调速(加/减速)、`k` 停止。

### 速度模式

`config/hero_chassis_controller.yaml` 中 `velocity_mode` 可选：

- `chassis`：`/cmd_vel` 视为底盘坐标系速度（键盘操控用这个）
- `world`：`/cmd_vel` 视为世界坐标系速度，通过 tf 转换到底盘系

### 代码规范

遵循 ROS C++ Style Guide，格式规则固化在 `.clang-format`：

```bash
clang-format -i -style=file src/*.cpp include/hero_chassis_controller/*.hpp
```