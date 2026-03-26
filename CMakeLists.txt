cmake_minimum_required(VERSION 3.10)

project(odometry C)

add_library(odometry src/odometry.c)

target_include_directories(odometry PUBLIC include)

add_executable(basic_example examples/basic_example.c)

target_link_libraries(basic_example odometry m)