cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

project(custom)

FIND_PACKAGE(Boost 1.65 COMPONENTS system REQUIRED)
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIR})

include_directories("../message")
add_library(messageLib ../message/Message.pb.cc)

ADD_EXECUTABLE(serverMain serverMain.cpp)

TARGET_LINK_LIBRARIES(serverMain LINK_PUBLIC ${Boost_LIBRARIES} messageLib protobuf)

ADD_EXECUTABLE(clientMain clientMain.cpp)

TARGET_LINK_LIBRARIES(clientMain LINK_PUBLIC ${Boost_LIBRARIES} pthread messageLib protobuf)
