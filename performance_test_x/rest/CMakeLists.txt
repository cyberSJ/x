cmake_minimum_required(VERSION 3.10)
project(rest)

set(Casablanca_LIBRARIES "-lboost_system -lcrypto -lssl -lcpprest")

include_directories("../message")
add_library(messageLib ../message/Message.pb.cc)

add_executable(serverMain serverMain.cpp)
target_link_libraries(serverMain ${Casablanca_LIBRARIES} messageLib protobuf)

add_executable(clientMain clientMain.cpp)
target_link_libraries(clientMain ${Casablanca_LIBRARIES} messageLib protobuf)
