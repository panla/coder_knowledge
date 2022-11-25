# 构建 ROS 软件包

[toc]

构建软件包及使用的工具链

## 0 doc

<https://wiki.ros.org/cn/ROS/Tutorials/BuildingPackages>

## 1 cmd

```bash
source /opt/ros/<distro>/setup.bash
```

### 1.1 catkin_make

```bash
catkin_make [make_targets] [-DCMAKE_VARIABLES=...]

cd ~/workspaces

# 构建src目录下的所有catkin项目
catkin_make

catkin_make install

# 指定 src
catkin_make --source src
```
