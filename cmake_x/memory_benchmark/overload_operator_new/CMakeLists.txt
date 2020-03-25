cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(overload_operator_new)

#set(CMAKE_BUILD_TYPE Debug)
#set(CMAKE_BUILD_TYPE RelWithDebInfo)

set(TOP_DIR ${PROJECT_SOURCE_DIR})
set(MY_LIBRARY MyLibrary)
add_library(${MY_LIBRARY} ${TOP_DIR}/MyClass.cpp)
add_executable(overloadTest ${TOP_DIR}/main.cpp)
target_link_libraries(overloadTest ${MY_LIBRARY})
