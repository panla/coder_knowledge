# 创建ros软件包

[toc]

## 0 doc

<https://wiki.ros.org/cn/ROS/Tutorials/CreatingPackage>

## 1 软件包的组成

### 1.1

- 这个包必须有一个符合 catkin 规范的 package.xml 文件
  - 这个文件提供该软件包的元信息
- 这个包必须有一个 catkin 版本的 CMakeLists.txt 文件
  - 如果是 Catkin 元包的话，则需要一个 CMakeLists.txt 文件的相关样板
- 每个包必须有自己的目录
  - 同一目录下不能嵌套或者多个包存在

```text
package/
    CMakeLists.txt
    package.xml
```

## 2 catkin 工作空间中的软件包

### 2.1 dir

```text
workspace/
    src/
        CMakeList.txt
        package/
            CMakeLists.txt
            package.xml
        package/
            CMakeLists.txt
            package.xml
    build/
    devel/
```

### 2.2 创建工作空间 workspace

```text
mkdir -p ~/project

cd ~/project

catkin_make
# 指定Python3
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
catkin_make -DPYTHON_EXECUTABLE=/usr/local/bin/python3

source devel/setup.bash

echo $ROS_PACKAGE_PATH
```

## 3 创建 catkin 软件包

```bash
mkdir -p workspaces/src

cd workspaces

catkin_make

cd src

# catkin_create_pkg <package_name> [depend1] [depend2]
catkin_create_pkg beginner_tutorials std_msgs rospy roscpp
# 创建一个名为 beginner_tutorials 的文件夹
# 依赖 std_msgs, rospy, roscpp
# 包含 package.xml 和 CMakeLists.txt 文件，
# 这两个文件已部分填写了创建 软件包时提供的信息

```

## 4 使工作空间配置文件生效

```bash
cd ~/workspaces

catkin_make

. ~/workspaces/devel/setup.bash
```

## 5 软件包依赖关系

### 5.1 一级依赖

`catkin_create_pkg` 创建包时指定的依赖，存储于 package.xml 中

```bash
rospack depends1 beginner_tutorials

# std_msgs
# rospy
# roscpp
```

### 5.2 间接依赖

一个依赖包有它自己的依赖关系

```bash
rospack depends1 rospy

# genpy
# roscpp
# ...
```

### 5.3 递归检测依赖

```bash
rospack depends beginner_tutorials
```

## 6 自定义软件包

### 6.1 package.xml

#### 6.1.1 example

```xml
<?xml version="1.0"?>
<package format="2">
  <name>hello_ros</name>
  <version>0.0.0</version>

   ...

  <!-- The export tag contains other, unspecified, tags -->
  <export>
    <!-- Other tools can request additional information be placed here -->
  </export>
</package>
```

#### 6.1.2 标签

描述标签 description

```xml
<description>The Package</description>
```

维护者标签 maintainer

```xml
<maintainer email="user@user.com">user</maintainer>
```

许可证标签 license

```xml
<license>MIT</license>
```

依赖项标签 `buildtool_depend` `build_depend`

```xml
<!-- catkin 中默认提供的 -->
<buildtool_depend>catkin</buildtool_depend>

<build_depend>roscpp</build_depend>

<exec_depend>roscpp</exec_depend>
```

### 6.2 CMakeLists.txt
