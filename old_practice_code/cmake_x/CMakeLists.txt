cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(cmake_x)
enable_testing()

if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU" OR
    "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    set(warnings "-Wall -Wextra -Werror")
elseif ("${CMAKE_CXX_COMPILER_ID}"  STREQUAL "MSVC")
    set(warnings "/W4 /WX /EHsc")
endif()

if (NOT CONFIGURED_ONCE)
    set(CMAKE_CXX_FLAGS "${warnings}" CACHE STRING 
    "Flgas used by the compiler during all build types." 
    FORCE)
endif()

# Not necessary to include_directories() for CMAKE_CURRENT_SOURCE_DIR 
# right now.... it doesn't make any difference...
#include_directories(${CMAKE_CURRENT_SOURCE_DIR})
message(STATUS "current source directory: " ${CMAKE_CURRENT_SOURCE_DIR})

# Set the top directory for reference further down this file.
set(TOP_DIR ${PROJECT_SOURCE_DIR})
message(STATUS "project source directory: " ${PROJECT_SOURCE_DIR})

# run CMakeLists.txt in the following sub directory.
# add_subdirectory() is only for 1 directory
add_subdirectory(todo)
add_subdirectory(mock)

# Adding gmock as a subdirectory makes. Adding gtest before adding gmock 
# gives an error!
#add_subdirectory(gtest-1.7.0)
add_subdirectory(gmock-1.7.0)   
message(STATUS "cmake binary dir: " ${CMAKE_BINARY_DIR})

# Doesn't need to include these directories. Already included in the 
# GooglTests's CMakeLists.txt
#message(STATUS "gtest source dir: " ${gtest_SOURCE_DIR})
#include_directories(
#   ${gtest_SOURCE_DIR}/gtest-1.7.0/include ${gtest_SOURCE_DIR})

# Include the header file location.
# include_directores() command is enough to find the headers.
include_directories(${TOP_DIR}/include)

# Include the gmock.cmake custom .cmake file I made.
list(APPEND CMAKE_MODULE_PATH ${TOP_DIR}/cmake/modules)
include(gmock)

# Use the custom-made function from the gmock.cmake module.
add_gmock_test(main main.cpp)
# Need to call this funciton here (and not inside add_gmock_test()) because
# every target depends on different libraries and add_gmock_test() can
# fullfill that requirement.
target_link_libraries(main todoLibrary)

# This section was used when add_gmock_test() was not studied.
#add_executable(main main.cpp)
#target_link_libraries(main 
#                      todoLibrary
#                      #gtest    # can link either with gtest or gtest_main
#                      gmock)    # If gmock is linked, you don't have to 
#                                # link gtest becuase gtest is actualy a 
#                                # part of gmock
#
#add_test(todoTest main)

# I think CONFIGURED_ONCE is on when this file finishes.
if (CONFIGURED_ONCE)
    message(STATUS "configured!")
elseif (NOT CONFIGURED_ONCE)
    message(STATUS "not configured...")
endif()
