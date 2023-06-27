# 编写 ROS Python Makefle

[TOC]

## 0 doc

<https://wiki.ros.org/rospy_tutorials/Tutorials/Makefile>

## 1 介绍

### 1.1 消息和服务

message and srv

### 1.2 安装脚本和导出模块

install scripts and exporting modules

```text
workspace
    src
        my_pkg
            bin
                hello
            src
                tutorial_package 此处应和 my_pkg 相同
                    __init__.py
                    hello.py
        setup.py

```

`workspace/src/my_pkg/src/my_package/hello.py`

```python
#! /usr/bin/env python
# -*- coding=utf-8 -*-

def day(name):
    print('hello', name)
```

`workspace/src/my_pkg/bin/hello`

```python
#! /usr/bin/env python
# -*- coding=utf-8 -*-

import my_package.hello

if __name__ == '__main__':
    my_package.hello.say('my friend')
```

```bash
# 增加可执行权限

chmod u+x bin/hello
```

```python
# 使用 distutils（不建议使用setuptools，因为它会在您的 src 文件夹中生成文件）
# 但是这种 setup 方式未来会从 Python 中删除
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup


# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['my_package'],
    package_dir={'': 'src'},
)

setup(**setup_args)
```

### 1.3 CMakeLists

```CMake
cmake_minimum_required(VERSION 3.0.2)
project(my_pkg)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  std_msgs
)

catkin_python_setup()

add_message_files(
  FILES
  Person.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  Say.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES my_pkg
    CATKIN_DEPENDS message_runtime message_generation rospy
#    DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS bin/hello 
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

```

### 1.4 and package.xml

```xml
  <buildtool_depend>catkin</buildtool_depend>

  <build_depend>rospy</build_depend>
  <build_depend>std_msgs</build_depend>
  <build_depend>message_generation</build_depend>

  <build_export_depend>rospy</build_export_depend>
  <build_export_depend>std_msgs</build_export_depend>
  <build_export_depend>message_generation</build_export_depend>

  <exec_depend>rospy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>message_runtime</exec_depend>
```
