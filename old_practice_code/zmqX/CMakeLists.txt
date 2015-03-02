CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
set(CMAKE_CXX_FLAGS "-std=c++11")

include_directories(cppzmq)
add_executable(zmqHelloWorldServerX zmqHelloWorldServerX.cpp)
target_link_libraries(zmqHelloWorldServerX zmq)

add_executable(zmqHelloWorldClientX zmqHelloWorldClientX.cpp)
target_link_libraries(zmqHelloWorldClientX zmq)

message(STATUS ${PROJECT_SOURCE_DIR})
message(STATUS ${CMAKE_SOURCE_DIR})
message(STATUS ${CMAKE_INCLUDE_PATH})
