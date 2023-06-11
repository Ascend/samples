cmake_minimum_required(VERSION 3.5.1)
project(cann_demo)

# OpenCV
find_package(OpenCV 4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# MobileNetV1
add_executable(mobilenetv1 mobilenetv1.cpp)
target_link_libraries(mobilenetv1 ${OpenCV_LIBS})
