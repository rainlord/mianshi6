# 声明要求的 cmake 最低版本
cmake_minimum_required(VERSION 2.8)

# 声明一个 cmake 工程
project(HelloSLAM)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

# 设置编译模式
set(CMAKE_BUILD_TYPE "Debug")

# 添加可执行程序调用hello库中函数
add_executable(useHello1 useHello1.cpp)
# 将库文件链接到可执行程序上
target_link_libraries(useHello1  -lpthread)


