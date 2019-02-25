cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(lidar_object_detection)
set (CMAKE_CXX_STANDARD 11)



find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
if (NOT OpenCV_FOUND)
    message(FATAL_ERROR "opencv needed but not found.")
    else()
    message(STATUS ${OpenCV_CONFIG_PATH})
endif()



include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (lidar_object_detect lidar_object_detect.cpp)
target_link_libraries (lidar_object_detect ${PCL_LIBRARIES} glog)
