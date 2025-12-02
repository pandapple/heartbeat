cmake_minimum_required(VERSION 3.10)
project(Heartbeat3D)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找PCL库
find_package(PCL 1.8 REQUIRED)

# 包含PCL头文件目录
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# 添加可执行文件
add_executable(heartbeat heartbeat.cpp)

# 链接PCL库
target_link_libraries(heartbeat ${PCL_LIBRARIES})

