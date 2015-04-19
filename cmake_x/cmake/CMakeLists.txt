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

# Running some executable in the directory where the executable is located.
execute_process(COMMAND ${TOP_DIR}/print_hello)
message(STATUS "install prefix     = " ${CMAKE_INSTALL_PREFIX})
set(CMAKE_INSTALL_PREFIX ${TOP_DIR}/install)
message(STATUS "new install prefix = " ${CMAKE_INSTALL_PREFIX})
# new install prefix = ${TOP_DIR}/install (not ${TOP_DIR}/install/usr/local as
# written in the cmake documentation).

# Run some executable that installs a whole another cmake project. Since the
# project is intended to be a totally independent project than this one, the
# install path of the external project should not interfere with this current
# project. WORKING_DREICTORY also affects the location where the COMMAND is
# run.
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

# Use macro to set some variable with multiple values.
macro(set_some_var target_var var1 var2 var3)
    # You cannot set one of the arguments using the other arguments.
    set(target_var ${var1} ${var2} ${var3})
    message(STATUS "target var = " ${target_var})
    set(var_using_argx ${ARGV1} ${ARGV2} ${ARGV3})
    message(STATUS "var using arg" ${ARGC} " = " ${var_using_argx})
    message(STATUS "remaining unused vars = " ${ARGN})
endmacro()
set_some_var(some_var "passing " "multiple " "values " 
                      "leftoever " "arguments ")
message(STATUS "some_var after marco = " ${some_var})
# some_var is empty because you cannot set one of the arguments using the other
# arguments.

# Test STREQUAL
set(some_string_1 "some_string_1")
set(some_string_2 "some_string_2")
set(some_string_3 ${some_string_1})
set(some_string_4 ${some_string_1} " " ${some_string_2})

if(NOT some_string_1 STREQUAL "some_string_1")
    message(WARNING "some_string_1 not equal to each other")
endif()
if(NOT some_string_1 STREQUAL some_string_3)
    message(WARNING "some_string_1 not equal to some_string_3")
endif()
# These are not equal because exapanding some_string_4 returns a multiple
# argument and if() statement does not like that.
#if(NOT some_string_4 STREQUAL "some_string_1 some_string_2")
#    message(WARNING "4 4 not equal")
#endif()
#if(NOT ${some_string_4} STREQUAL "some_string_1 some_string_2")
#    message(WARNING "4 4 not equal (expanded version)")
#endif()
# This doesn't work either because each item in the list is separated by ";" (I
# originally though ";" wasn't there.
#if(NOT "${some_string_4}" STREQUAL "some_string_1 some_string_2")
#    message(STATUS "${some_string_4} = " "${some_string_4}")
#    message(WARNING "4 4 not equal (double quote expanded version left)")
#endif()
# But this makes sense because they are both delimited by ";".
if(NOT some_string_4 STREQUAL "${some_string_4}")
    message(WARNING "4 4 not equal (double quote expanded version right)")
else()
    message(STATUS "${some_string_4} = " "${some_string_4}")
endif()

# Replace a part of the a string
string(REPLACE "some" "other" output_string ${some_string_4})
message(STATUS "output_string = " ${output_string})
# output_string = other_string_1 other_string_2

# Determine a length of a list
list(LENGTH some_string_4 len_some_string_4)
message(STATUS "length of a list = " ${len_some_string_4})
# length of a list = 3

# Get a part of a list. CMake is 0-based index
list(GET some_string_4 0 2 partial_list)
message(STATUS "partial list = " ${partial_list})
# partial list = some_string_1som_string_2

# Try to find a part of list. Return -1 if not found.
# the <value> argument must match exactly with the partial.
list(FIND some_string_4 "some_string_2" partial_found)
message(STATUS "partial found? = " ${partial_found})
# partial found? = 2
