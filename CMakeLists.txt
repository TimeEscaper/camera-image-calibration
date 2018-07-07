cmake_minimum_required(VERSION 3.10)
project(camera_image_calibration)

set(CMAKE_CXX_STANDARD 11)

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
add_executable( camera_image_calibration main.cpp )
target_link_libraries( camera_image_calibration ${OpenCV_LIBS} )