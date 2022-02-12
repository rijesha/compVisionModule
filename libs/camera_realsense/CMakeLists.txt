# camera_realsense/CMakeLists.txt

cmake_minimum_required(VERSION 3.15)

project(camera_realsense VERSION 0.1.0
        DESCRIPTION "Camera Realsense library"
        LANGUAGES CXX)

add_library(${PROJECT_NAME} STATIC)
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)
target_sources(${PROJECT_NAME}
    PUBLIC
        include/camera_realsense.h
    PRIVATE
        src/camera_realsense.cpp
)

# Find librealsense2 installed package
find_package(realsense2 REQUIRED)


target_link_libraries(${PROJECT_NAME} ${realsense2_LIBRARY})

target_include_directories(${PROJECT_NAME} PUBLIC include)

# Enable C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
