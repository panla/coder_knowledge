# 使用 rqt_console 和 roslaunch

[TOC]

## 0 doc

<https://wiki.ros.org/cn/ROS/Tutorials/UsingRqtconsoleRoslaunch>

## 1 rqt_console rqt_logger_level

```bash
apt install ros-<distro>-rqt ros-<distro>-rqt-common-plugins ros-<distro>-turtlesim
```

- rqt_console 连接到ROS的日志框架，以显示节点的输出信息
- rqt_logger_level 允许在节点运行时改变输出信息的详细级别

## 3 roslaunch

启动定义在 launch 文件中的节点

```bash
source devel/setup.bash

roslaunch [package] [filename.launch]
```

### 3.1

```xml
<launch>

    <group ns="namespace">

        <node pkg="packageName" name="guardian_warn" type="guardian_warn_new" output="screen" cwd="node"></node>

    </group>

    <arg name="age" default="5" />
    <param name="school/age" value="$(arg age)" type="int" />

    <node pkg="strategy" name="strategy" type="strategy.py" output="screen" cwd="node"></node>

</launch>
```

较高层级结构，包含并覆盖另一个文件

```xml
<launch>

    <include file="$(find person)/config/person.launch" />

        <arg name="age" default="2" />

    </include>

</launch>
```
