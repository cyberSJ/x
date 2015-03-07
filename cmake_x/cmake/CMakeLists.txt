cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(cmake_x)
# I can't make LANGUAGES to work. The following line generates error.
# project(cmake_x LANGUAGES CXX)
message(STATUS "project name: " ${PROJECT_NAME})
message(STATUS "project source dir: " ${cmake_x_SOURCE_DIR})
message(STATUS "project binary dir: " ${PROJECT_BINARY_DIR})
# Binary directory is where you put the build artifacts.

# How does CMake contain multiple values in one variable?:
set(one_variable "var1" "var2" "var3")
message(STATUS "one variable contains: " ${one_variable})
# Result: -- one variable contains: var1var2var3

# PROJECT_SOURCE_DIR is the directory where the CMakeLists.txt file is.
set(TOP_DIR ${PROJECT_SOURCE_DIR}/..)
message(STATUS "top dir: " ${TOP_DIR})

# Does CMake transfer the value that has been set() down to other CMakeLists.txt
# files that are invoked later in this top CMakeLists.txt file?: yes
set(transferrable_variable "I can see this variable in submodule!")
set(CMAKE_MODULE_PATH ${TOP_DIR}/sub_directory/cmake)
include(some_module)
add_subdirectory(${TOP_DIR}/sub_directory/cmake ${TOP_DIR}/sub_directory/build)
# Question: How do you not manuaully specify **/cmake when you include a
# CMakeLists.txt?

# Trying to set different data types in one variable
set(mixed_variable 1 "one")
message(STATUS "mixed_variable: " ${mixed_variable})
# Result: -- mixed_variable: 1one

add_executable(test ${TOP_DIR}/test.cpp)
