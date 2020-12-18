# tcp_server_client/CMakeLists.txt

cmake_minimum_required(VERSION 3.15)

project(tcp_server_client VERSION 0.1.0
        DESCRIPTION "tcp_server_client library"
        LANGUAGES CXX)

add_library(${PROJECT_NAME} STATIC)
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)

find_package (Threads)

target_sources(${PROJECT_NAME}
    PRIVATE
        include/tcp_server_client/tcp_server.h
        include/tcp_server_client/tcp_client.h
        include/tcp_server_client/server_observer.h
        include/tcp_server_client/client.h
        include/tcp_server_client/client_observer.h
        include/tcp_server_client/pipe_ret_t.h
        src/tcp_client.cpp
        src/tcp_server.cpp
        src/client.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC include)

target_link_libraries (tcp_server_client ${CMAKE_THREAD_LIBS_INIT})

