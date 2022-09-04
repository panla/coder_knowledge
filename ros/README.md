# README

## 简介

ROS遵循BSD开源许可协议。

ROS (Robot Operating System, 机器人操作系统) 提供一系列程序库和工具以帮助软件开发者创建机器人应用软件。

它提供了硬件抽象、设备驱动、函数库、可视化工具、消息传递和软件包管理等诸多功能。

它提供了操作系统应有的服务，包括硬件抽象，底层设备控制，常用函数的实现，进程间消息传递，以及包管理。

它也提供用于获取、编译、编写、和跨计算机运行代码所需的工具和库函数。

一个 ROS 系统由许多独立的节点组成，每个节点都使用发布/订阅消息模型与其他节点进行通信。

例如，特定传感器的驱动程序可能被实现为一个节点，它以消息流的形式发布传感器数据。这些消息可以被任意数量的其他节点使用，包括过滤器、记录器以及更高级别的系统，例如导航、寻路等。

- ROS 运行时的"蓝图"是一种基于ROS通信基础结构的松耦合点对点进程网络
- ROS 实现了几种不同的通信方式，包括
  - 基于 RPC 样式通信的 **服务services** 机制 <https://wiki.ros.org/Services>
  - 基于异步流媒体数据的 **话题 topics** 机制 <https://wiki.ros.org/Topics>
  - 用于数据存储的 **参数服务器 parameter server** <https://wiki.ros.org/Parameter%20Server>

## docs

- introduction <https://wiki.ros.org/cn/ROS/Introduction>
- tutorials <https://wiki.ros.org/cn/ROS/Tutorials>
  - first environment <https://wiki.ros.org/cn/ROS/Tutorials/InstallingandConfiguringROSEnvironment>
  - file system <https://wiki.ros.org/cn/ROS/Tutorials/NavigatingTheFilesystem>

## docker use tutorials

```bash
# Get Image
docker pull ros:noetic-robot

# generate container
docker run -it ros:noetic-robot

# activate env
source ros_entrypoint.sh

## ROS2
ros2 topic list

# ROS1
roscore

# new create workspace
make -p ~/catkin_ws/src && cd ~/catkin_ws/ && catkin_make

# 工作区被安装脚本正确覆盖
echo $ROS_PACKAGE_PATH
```
