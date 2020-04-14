# argparse/CMakeLists.txt

cmake_minimum_required(VERSION 3.15)

project(argparse VERSION 0.1.0
        DESCRIPTION "argparse library"
        LANGUAGES CXX)

add_library(${PROJECT_NAME} STATIC)
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)
target_sources(${PROJECT_NAME}
    PUBLIC
        argparse.hpp
)

target_include_directories(${PROJECT_NAME} PUBLIC ./)
