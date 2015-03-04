cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(cmake_x)
#I can't make LANGUAGES to work. The following line generates error.
#project(cmake_x LANGUAGES CXX)
message(STATUS "project name: " ${PROJECT_NAME})
message(STATUS "project source dir: " ${cmake_x_SOURCE_DIR})

set(TOP_DIR ${PROJECT_SOURCE_DIR}/..)
message(STATUS "top dir: " ${TOP_DIR})

add_executable(test ${TOP_DIR}/test.cpp)
