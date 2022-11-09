# 创建 ROS消息和服务

[toc]

## 0

<https://wiki.ros.org/cn/ROS/Tutorials/CreatingMsgAndSrv>

创建和构建 msg 和 srv 文件，使用 rosmsg rossrv roscp

## 1 msg srv description

- msg 消息
  - msg 文件用于描述ROS消息的字段，用于为不同编程语言编写的消息生成源代码
  - 类型
    - Header 含有时间戳和ROS中的坐标帧信息
    - int8 int16 int32 int64
    - uint8 uint16 uint32 uint64
    - float32 float64
    - string
    - time duration
    - 其他 msg 文件
    - variable-length array[] fixed-length array[C]
- srv 服务
  - 一个 srv 文件描述一个服务，由请求和响应组成
  - `---` 上面是请求，下面是响应

```text
Header header
string name
uint8 age
LocalizationStatus rtk_status
```

```text
string name
---
uint8 age
```

## 2 msg

### 2.1 创建 msg

package.xml

```xml
<build_depend>message_generation</build_depend>
<exec_depend>message_runtime</build_depend>
```

CMakeLists.txt

```text
# 增加依赖
find_package(catkin REQUIRED COMPONENTS
    ...
    message_generation
)

# 确保导出消息的运行时依赖关系
catkin_package(
      ... message_runtime
)

# 增加msg
add_message_files(
    ...
    Person.msg
)

# 手动添加 msg文件后，我们要确保CMake知道何时需要重新配置项目
generate_messages(
    ...
    std_msgs
)
```

### 2.2 use rosmsg

```bash
rosmsg show [message type]
```

## 3 srv

### 3.1 创建 srv

package.xml

```xml
<build_depend>message_generation</build_depend>
<exec_depend>message_runtime</build_depend>
```

CMakeLists.txt

```text
# 增加依赖
find_package(catkin REQUIRED COMPONENTS
    ...
    message_generation
)

# 确保导出消息的运行时依赖关系
catkin_package(
      ... message_runtime
)

# 增加srv
add_service_files(
    ...
    Person.srv
)

```

### 3.2 use rossrv

```bash
rossrv show <service type>
```
