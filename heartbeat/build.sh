#!/bin/bash

# 创建构建目录
mkdir -p build
cd build

# 运行CMake配置
cmake ..

# 编译
make

echo "编译完成！运行 ./build/heartbeat 来启动程序"

