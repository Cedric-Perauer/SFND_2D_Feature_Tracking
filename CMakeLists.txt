cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")
<<<<<<< HEAD
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "-Ofast")
set(OpenCV_DIR /home/cedric/Downloads/opencv_4.1/opencv/build)
=======

>>>>>>> 71003a7c9c2f3092361f59224fe05b71e559f4f6

project(camera_fusion)

find_package(OpenCV 4.1 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})

# Executable for create matrix exercise
add_executable (2D_feature_tracking src/matching2D_Student.cpp src/MidTermProject_Camera_Student.cpp)
target_link_libraries (2D_feature_tracking ${OpenCV_LIBRARIES})
