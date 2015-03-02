cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(Todo)

set(TOP_DIR ${PROJECT_SOURCE_DIR}/..)

# Include direcotry that contains header files.
include_directories(${TOP_DIR}/include)

# In the build directory (since that is where this cmake is called),
# create the libtodoLibrary.so
add_library(todoLibrary SHARED Todo.cpp)
message(STATUS "end of Todo CMakeLists.txt")
