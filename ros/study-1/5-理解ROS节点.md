# 理解 ROS 节点

[toc]

<https://wiki.ros.org/cn/ROS/Tutorials/UnderstandingNodes>

## 1 tutorials

```bash
apt install ros-noetic-ros-tutorials
```

## 2 图概念速览

计算图是一个由ROS进程组成的点对点网络，这些进程能够共同处理数据

- 节点: 一个可执行文件，通过 ROS 与其他节点通信
- 消息: 订阅或发布话题时所使用的 ROS 数据类型
- 话题: 节点可以将消息发布到话题，或通过订阅话题来接收消息
- 主节点: ROS 的命名服务
- rosout: 在 ROS 中相当于 stdout/stderr
- roscore: 主节点 + rosout + 参数服务器

## 3 节点

使用 ROS 客户端库于其他节点通信，可以发布或订阅话题，也可以提供或使用服务

## 4 客户端库

rospy, roscpp

## 5 roscore

运行所有 ROS 程序前首先要运行的命令

```bash
roscore
```

## 6 rosnode

```bash
rosnode list

rosnode info /rosout
```

## 7 rosrun

运行给定的软件包中的节点

```bash
rosrun [package_name] [node_name]
```
