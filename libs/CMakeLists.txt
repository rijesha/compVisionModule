cmake_minimum_required(VERSION 3.15)

add_subdirectory(argparse)
add_subdirectory(position_controller)
add_subdirectory(camera-v4l2)
add_subdirectory(mavlink-interface)
add_subdirectory(common)
if(BUILD_REALSENSE)
	add_subdirectory(camera_realsense)
endif()

add_subdirectory(aruco_processor)
