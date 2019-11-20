cmake_minimum_required(VERSION 3.5)

project(ems_logger_2 LANGUAGES CXX)
find_package (Threads)

find_package( OpenCV REQUIRED )

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_library(LIB_RT  rt)
add_executable(ems_logger_2 src/main.cpp src/serial.cpp src/util.cpp src/Zed_cpu_2.cpp src/StereoCamera.cpp)
target_link_libraries (ems_logger_2 ${CMAKE_THREAD_LIBS_INIT} ${LIB_RT} ${OpenCV_LIBS})
