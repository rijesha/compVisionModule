# CMakeLists.txt

cmake_minimum_required(VERSION 3.15)

project(compVisionModule VERSION 0.1.0
		DESCRIPTION "Computer Vision Moduel for ARUCO"
		LANGUAGES C CXX)

set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wno-address-of-packed-member")

IF(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE "Release"
			CACHE STRING "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel."
			FORCE)
ENDIF()

option(BUILD_GROUND "Build Ground Component" OFF)
if(BUILD_GROUND)
	message(INFO "Building Ground Component.")
endif()

option(BUILD_REALSENSE "Build Realsense" OFF)
if(BUILD_REALSENSE)
	message(INFO "Building realsense.")
endif()

# set cmake directory
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# third party / external libraries
add_subdirectory(third_party)

# internal libraries
add_subdirectory(libs)

if(BUILD_REALSENSE)
	add_subdirectory(uav_sensor_deploy)
endif()

add_subdirectory(aruco_test_data_collection)
add_subdirectory(simulated_target_system)
if(BUILD_GROUND)
	add_subdirectory(ground_component)
endif()
