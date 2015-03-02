CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

add_executable(mainX mainX.cpp)
add_library(threadPoolX threadPoolX.cpp)
target_link_libraries(mainX pthread threadPoolX)
set(CMAKE_CXX_FLAGS "-std=c++11 -g -Wall")
