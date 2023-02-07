# [Optional] [Advice] 指定 cmake 最小版本
cmake_minimum_required(VERSION 3.22)

# 指定编译器
# set(CMAKE_C_COMPILER /usr/bin/gcc)
# set(CMAKE_CXX_COMPILER /usr/bin/g++)

# 设置编译方式
SET(CMAKE_BUILD_TYPE Release)
# 设置 C++ 版本
set(CMAKE_CXX_STANDARD 14)
# 设置 CMake传给C++编译器的编译选项
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g ${CMAKE_CXX_FLAGS}")

# [Optional] [Advice] 工程名程
# 自动获得 hello_BINARY_DIR hello_SOURCE_DIR 两个变量 和等价的 PROJECT_BINARY_DIR PROJECT_SOURCE_DIR
project(hello)

# 设置生成的可执行文件保存路径
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
# 设置生成的链接库保存路径
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
# 设置源代码路径
set(SOURCE_CODE_DIR ${PROJECT_SOURCE_DIR}/src)

# 追加数组变量
# set(SRC_LIST main.cpp test.cpp)
# set(SRC_LIST ${SRC_LIST} hello.cpp)
# list(APPEND SRC_LIST person.cpp)
# list(REMOVE_ITEM SRC_LIST person.cpp)

# 添加非标准的共享库搜索路径，链接库搜索目录
link_directories(
    ${PROJECT_SOURCE_DIR}/lib
)
# 设置头文件目录，以使系统找到对应的头文件
include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/common/idgen
)
# 把目标文件与库文件进行链接
# target_link_libraries(hello libperson.so)

# 添加子项目
# add_subdirectory("")

# 搜索当前路径下所有 cpp 文件，并把结果保存在变量 SRC_LIST
# aux_source_directory(. SRC_LIST)

# 查找指定库文件，文件，包，路径，
# find_library()
# find_file()
# find_package()
# find_path()
# find_program()

# 生成动态共享库
# [name] [type] [path]
# 把多个源文件生成一个库
add_library(
    hello SHARED ${SOURCE_CODE_DIR}/hello.cpp
)
add_library(
    person SHARED ${SOURCE_CODE_DIR}/person.cpp
)
add_library(
    dog SHARED ${SOURCE_CODE_DIR}/dog.cpp
)
add_library(
    fib SHARED ${SOURCE_CODE_DIR}/fib.c
)
# add_library(
#     SnowFlake SHARED ${SOURCE_CODE_DIR}/SnowFlake.c
# )

# 生成静态库
# add_library(
#     person STATIC ${SOURCE_CODE_DIR}/person.cpp
# )

# 生成可执行文件，用到的 cpp 文件都写上
add_executable(
    main
    ${SOURCE_CODE_DIR}/main.cpp
    ${SOURCE_CODE_DIR}/hello.cpp
    ${SOURCE_CODE_DIR}/person.cpp
)
add_executable(
    cat
    ${SOURCE_CODE_DIR}/cat.c
)
# add_executable(
#     fib
#     ${SOURCE_CODE_DIR}/fib.c
# )

# 预定义变量
# PROJECT_NAME                通过 project 命令定义的项目名称
# PROJECT_SOURCE_DIR          项目根目录
# PROJECT_BINARY_DIR          运行 cmake 命令的目录，一般在 ${PROJECT_SOURCE_DIR}/build
# CMAKE_CURRENT_SOURCE_DIR    当前处理的 CMakeLists.txt 所在的目录
# CMAKE_CURRENT_BINARY_DIR    target 编译目录
# CMAKE_CURRENT_LIST_DIR      CMakeLists.txt 的完整路径
# CMAKE_CURRENT_LIST_LINE     当前所在行
# CMAKE_CURRENT_LIST_FILE
# CMAKE_MODULE_PATH
# EXECUTABLE_OUTPUT_PATH      可执行文件存放位置
# LIBRARY_OUTPUT_PATH         链接库存放位置
