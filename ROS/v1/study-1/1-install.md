# install

[TOC]

<https://wiki.ros.org/cn/melodic/Installation/Ubuntu>

## 1 ros:melodic

### 1.1 安装

#### 1.1.1 增加源

```text
ubuntu18.04

edit /etc/apt/sources.list
deb https://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/ bionic main

sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

sudo apt update
```

```text
ubuntu20.04

edit /etc/apt/sources.list
deb https://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/ focal main

sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

sudo apt update
```

#### 1.1.2 选择版本

```text
# 完整桌面版
# 包含 ROS，rqt，rviz，机器人通用库，2D/3D 模拟器，导航以及 2D/3D 感知包
sudo apt install ros-melodic-desktop-full

# 桌面版
# 包含 ROS, rqt, rviz，机器人通用库
sudo apt install ros-melodic-desktop

# 基础包
# 包含 ROS，构建和通信库
sudo apt install ros-melodic-ros-base
```

### 1.2 init rosdep

```bash
sudo rosdep init
rosdep update
```

### 1.3 ENV

```bash
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc

source ~/.bashrc
```

### 1.4 构建工厂依赖

rosinstall

```bash
sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential
```

### 1.4 安装库

```text
sudo apt install ros-melodic-<Package>

# 展示 bag 主题消息，图形化
sudo apt install ros-melodic-plotjuggler-*
```
