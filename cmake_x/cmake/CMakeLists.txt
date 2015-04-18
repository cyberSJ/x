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
set(transferrable_variable "I can see this variable in submodule and "
    "CMakeLists.txt!")
set(CMAKE_MODULE_PATH ${TOP_DIR}/sub_directory/cmake)
include(some_module)
add_subdirectory(${TOP_DIR}/sub_directory/cmake ${TOP_DIR}/sub_directory/build)
# Question: How do you not manuaully specify **/cmake when you include a
# CMakeLists.txt?: You can't.

# Trying to set different data types in one variable
set(mixed_variable 1 "one")
message(STATUS "mixed_variable: " ${mixed_variable})
# Result: -- mixed_variable: 1one

add_executable(test ${TOP_DIR}/test.cpp)

# Running some executable in its directory.
execute_process(COMMAND ${TOP_DIR}/print_hello)
message(STATUS "install prefix     = " ${CMAKE_INSTALL_PREFIX})
set(CMAKE_INSTALL_PREFIX ${TOP_DIR}/install)
message(STATUS "new install prefix = " ${CMAKE_INSTALL_PREFIX})
# new install prefix = ${TOP_DIR}/install (not ${TOP_DIR}/install/usr/local as
# written in the cmake documentation).

# Run some executable that installs a whole another cmake project. Since the
# project is intended to be a totally independent project than this one, the
# install path of the external project should not interfere with this current
# project.
execute_process(COMMAND ./install_some_project
                WORKING_DIRECTORY ${TOP_DIR}/some_project/)

# Include .cmake module (even though I did it before)
list(APPEND CMAKE_MODULE_PATH ${TOP_DIR}/sub_direcotry/cmake)
include(some_module)
include(some_module_2)

# Find a program. Does it execut it as well?: No, it just prints out the
# location of the program
find_program(EXECUTE_PROCESS_EXE
             NAMES execute_some_process
             PATHS ${TOP_DIR}/some_project/build)
if (${EXECUTE_PROCESS_EXE} STREQUAL EXECUTE_PROCESS_EXE-NOTFOUND)
    message(STATUS "execute_some_process is NOT found.")
else()
    message(STATUS "execute_some_process found.")
    message(STATUS "result = " ${EXECUTE_PROCESS_EXE})
endif()

# Make EXECUTE_PROCESS_EXE variable appear only on "advanced" option in cmake
# gui.
mark_as_advanced(EXECUTE_PROCESS_EXE)
