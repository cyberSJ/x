cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# Need to specify these directory because my custom header and the 
# main.cpp are tyring to include the gmock.h. Although gtest can be found 
# without this include_directories() function (I don't know why it can 
# find gtest), but gmock can only be found by including all gmock & gtest 
# include directory. So I guess include_directories() are include files 
# only for the files directly used by this CMake file.
include_directories("${gmock_SOURCE_DIR}/include"
                    #"${gmock_SOURCE_DIR}"
                    "${gtest_SOURCE_DIR}/include")
                    #"${gtest_SOURCE_DIR}")

get_property(INC_DIR DIRECTORY PROPERTY INCLUDE_DIRECTORIES)
# Putting quotes around ${INC_DIR} allows the directory strings to be 
# seperated by a semicolon.
message(STATUS "include DIR: " "${INC_DIR}")

message(STATUS "working directory: " "${CMAKE_CURRENT_BINARY_DIR}")

# Make a custom function that does 3 things in 1 step.
# Basically, auto-registering a target to a CMAKE test with gtest/gmock
# capability.
function(add_gmock_test target)
    add_executable(${target} ${ARGN})
    target_link_libraries(${target} gmock)

# Add test to CMake test (independent with Google Test)
    add_test(${target}_test ${target})

# Automatically run the Google Test (not the CMake test) when the target is
# first built.
    add_custom_command(TARGET ${target}
                       POST_BUILD 
                       COMMAND ${target}
                       WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
                       COMMENT "Running ${target}"
                       VERBATIM)

endfunction()
