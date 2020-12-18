# CMakeLists.txt

cmake_minimum_required(VERSION 3.15)

project(compVisionModule VERSION 0.1.0
		DESCRIPTION "Computer Vision Moduel for ARUCO"
		LANGUAGES C CXX)

IF(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE "MinSizeRel"
			CACHE STRING "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel."
			FORCE)
ENDIF()

# set cmake directory
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# third party / external libraries
add_subdirectory(third_party)

# internal libraries
add_subdirectory(libs)

add_subdirectory(uav_sensor_deploy)
add_subdirectory(aruco_test_data_collection)
add_subdirectory(simulated_target_system)
