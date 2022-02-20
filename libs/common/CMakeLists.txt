# common/CMakeLists.txt

cmake_minimum_required(VERSION 3.15)

project(common VERSION 0.1.0
        DESCRIPTION "common library"
        LANGUAGES CXX)

add_library(${PROJECT_NAME} STATIC)
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)
target_sources(${PROJECT_NAME}
    PUBLIC
        common/cvm_argument_parser.hpp
        common/navigational_state.hpp
        common/configuration.h
    PRIVATE
        common/cvm_argument_parser.cpp
)

find_package(OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )

message(STATUS "OpenCV found, version: ${OpenCV_VERSION} in dir ${OpenCV_INCLUDE_DIRS}")

SET(ARUCO_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/aruco/src/ )
include_directories(${ARUCO_INCLUDE_DIRS})

target_include_directories(${PROJECT_NAME} PUBLIC ./)
target_link_libraries(${PROJECT_NAME}
PUBLIC
        argparse
        aruco
)

target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)
