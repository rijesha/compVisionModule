# uav_sensor_deploy/CMakeLists.txt

cmake_minimum_required(VERSION 3.15)

project(uav_sensor_deploy VERSION 1.0
        DESCRIPTION "uav_sensor_deploy Project"
        LANGUAGES C CXX ASM)


add_executable(${PROJECT_NAME} main.cpp)
target_sources(${PROJECT_NAME}
	PRIVATE
                states.hpp
)

find_package(OpenCV REQUIRED)
message(STATUS "OpenCV library status:")
message(STATUS "    config: ${OpenCV_DIR}")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

SET(ARUCO_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/aruco/src/ )
include_directories(${ARUCO_INCLUDE_DIRS})

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

target_link_libraries(${PROJECT_NAME} 
        PRIVATE 
           Threads::Threads
           common
           mavlink-interface
           position_controller
           camera_realsense
           ${OpenCV_LIBS}
           
        )
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)
