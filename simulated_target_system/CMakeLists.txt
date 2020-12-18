# simulated_target/CMakeLists.txt

cmake_minimum_required(VERSION 3.15)

project(simulated_target VERSION 1.0
        DESCRIPTION "simulated_target"
        LANGUAGES C CXX ASM)

add_executable(${PROJECT_NAME} main.cpp)
target_sources(${PROJECT_NAME}
	PRIVATE
)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

target_link_libraries(${PROJECT_NAME} 
        PRIVATE 
           Threads::Threads
           common
           mavlink-interface
           attitude_controller
           mavlink-server
        )
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_14)
