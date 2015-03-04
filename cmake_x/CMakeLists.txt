cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(cmake_x)
#I can't make LANGUAGES to work. The following line generates error.
#project(cmake_x LANGUAGES CXX)

message(STATUS ${PROJECT_NAME})
message(STATUS ${cmake_x_SOURCE_DIR})

add_executable(test test.cpp)
