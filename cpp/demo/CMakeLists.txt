cmake_minimum_required(VERSION 3.22)

# 工程名程
project(hello)

# 设置编译方式
SET(CMAKE_BUILD_TYPE Release)
# 设置 CMake传给C++编译器的编译选项
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g ${CMAKE_CXX_FLAGS}")

# 设置生成的可执行文件保存路径
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
# 设置生成的链接库保存路径
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
# 设置源代码路径
set(SOURCE_CODE_DIR ${PROJECT_SOURCE_DIR}/src)

# 设置头文件目录，以使系统找到对应的头文件
include_directories(
    ${PROJECT_SOURCE_DIR}/include
)

# 生成动态共享库，将之生成 .so
add_library(
    hello SHARED ${SOURCE_CODE_DIR}/hello.cpp
)
add_library(
    person SHARED ${SOURCE_CODE_DIR}/person.cpp
)

# 生成可执行文件，全部 cpp 文件都写上
add_executable(
    main ${SOURCE_CODE_DIR}/main.cpp ${SOURCE_CODE_DIR}/hello.cpp ${SOURCE_CODE_DIR}/person.cpp
)
