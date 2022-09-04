# 构建ros软件包

[toc]

<https://wiki.ros.org/cn/ROS/Tutorials/CreatingPackage>

## 1 软件包的组成

- 这个包必须有一个符合 catkin 规范的 package.xml 文件
  - 这个文件提供该软件包的元信息
- 这个包必须有一个 catkin 版本的 CMakeLists.txt 文件
  - 如果是 Catkin 元包的话，则需要一个 CMakeLists.txt 文件的相关样板
- 每个包必须有自己的目录
  - 统一目录下不能嵌套或者多个包存在

```text
package/
    CMakeLists.txt
    package.xml
```

## 2 catkin 工作空间中的软件包

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

## 3 构建 catkin 软件包

```bash
mkdir -p workspaces/src

cd workspaces

catkin_make

cd src

catkin_create_pkg beginner_tutorials std_msgs rospy roscpp
# catkin_create_pkg <package_name> [depend1] [depend2]
# 创建一个名为 beginner_tutorials 的文件夹
# 依赖 std_msgs, rospy, roscpp
# 包含 package.xml 和 CMakeLists.txt 文件，
# 这两个文件已部分填写了创建 软件包时提供的信息

```

package.xml

```text
<?xml version="1.0"?>
<package format="2">
  <name>hello_ros</name>
  <version>0.0.0</version>
  <description>The hello_ros package</description>

  <maintainer email="root@todo.todo">root</maintainer>
  <license>TODO</license>

  <!-- <url type="website">http://wiki.ros.org/hello_ros</url> -->

  <!-- <author email="jane.doe@example.com">Jane Doe</author> -->

  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>rospy</build_depend>
  <build_depend>std_msgs</build_depend>
  <build_export_depend>rospy</build_export_depend>
  <build_export_depend>std_msgs</build_export_depend>
  <exec_depend>rospy</exec_depend>
  <exec_depend>std_msgs</exec_depend>

  <!-- The export tag contains other, unspecified, tags -->
  <export>
    <!-- Other tools can request additional information be placed here -->
  </export>
</package>
```

## 4 使工作空间配置文件生效

```bash
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
rospack rospy

# genpy
# roscpp
# ...
```

### 5.3 递归检测依赖

```bash
rospack depends beginner_tutorials
```
