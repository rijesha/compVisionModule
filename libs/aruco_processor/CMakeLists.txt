# aruco_processor/CMakeLists.txt

cmake_minimum_required(VERSION 3.15)

project(aruco_processor VERSION 0.1.0
        DESCRIPTION "ArUco Processor library"
        LANGUAGES CXX)

find_package(OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )

message(STATUS "OpenCV found, version: ${OpenCV_VERSION} in dir ${OpenCV_INCLUDE_DIRS}")

SET(ARUCO_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/aruco/src/ )
include_directories(${ARUCO_INCLUDE_DIRS})

add_library(${PROJECT_NAME} STATIC)
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)
target_sources(${PROJECT_NAME}
    PUBLIC
        include/aruco_processor/aruco_dual_image_processor.h
        include/aruco_processor/aruco_position.h
	include/aruco_processor/aruco_processor.h
        include/aruco_processor/undistort_image.h
    PRIVATE
        src/aruco_position.cpp
	src/aruco_dual_image_processor.cpp
	src/aruco_processor.cpp
        src/undistort_image.cpp
)

target_link_libraries(${PROJECT_NAME}
PUBLIC
        position_controller
        aruco
)

target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC ./include)
