# uav_sensor_deploy/CMakeLists.txt

cmake_minimum_required(VERSION 3.15)

project(folder_analyzer VERSION 1.0
        DESCRIPTION "test data analysis"
        LANGUAGES C CXX ASM)


add_executable(${PROJECT_NAME} folder_analyzer.cpp)
add_executable(test_data_collection test_data_collection.cpp)

find_package(OpenCV REQUIRED)
message(STATUS "OpenCV library status:")
message(STATUS "    config: ${OpenCV_DIR}")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")
include_directories( ${OpenCV_INCLUDE_DIRS} )

SET(ARUCO_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/aruco/src/ )
include_directories(${ARUCO_INCLUDE_DIRS})

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

target_link_libraries(${PROJECT_NAME} 
        PRIVATE 
           -lboost_system
           common
           mavlink-interface
           attitude_controller
           ${OpenCV_LIBS}
           Threads::Threads
        )
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_14)


target_link_libraries(test_data_collection 
        PRIVATE 
           common
           mavlink-interface
           attitude_controller
           ${OpenCV_LIBS}
           Threads::Threads
           camera-v4l2
        )
target_compile_features(test_data_collection PRIVATE cxx_std_14)

