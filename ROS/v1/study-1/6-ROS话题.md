# ROS话题

[TOC]

## 0 doc

<https://wiki.ros.org/cn/ROS/Tutorials/UnderstandingTopics>

## 1 `rqt_graph`

`rqt_graph` 用动态的图显示系统中正在发生的事情，`rqt_graph` 是 rqt 程序包的一部份

```bash
apt install ros-<distro>-rqt
apt install ros-<distro>-reqt-common-plugins

rosrun rqt_graph rqt_graph

# 节点A--话题-->节点B
```

## 2 rostopic

获取话题信息

### 2.1 usage cmd

```bash
rostopic -h

    list    列出 活跃的 topic
    echo    打印实时消息
        rostopic echo topic
    pub     发布消息到 topic
    find    根据类型查找 topic
    info    打印活跃的 topic 信息
    delay   display topic 消息延迟
    type    打印 topic 或字段类型
    bw      display bandwidth
    hz      display 发送消息频率

rostopic command -h
```

## 3 ROS消息

### 3.1 rostopic type

```bash
rostopic type topic
    msg_type

rosmsg show msg_type
```

## 4 发布

```bash
rostopic pub [topic] [msg_type] [args]

rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]'

rostopic pub        发布命令
-1                  只发布一条
/turtle1/cmd_vel    指定 topic
geometry_msgs/Twist 使用的消息类型
--                  表明之后的参数都不是选项

-r 1                频率为1Hz
```

## 5 `rqt_plot`

`rqt_plot` 命令可以在滚动时间图上显示发布到某个话题上的数据
