cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(some_project)
set(TOP_DIR ${PROJECT_SOURCE_DIR}/..)

add_executable(execute_process ${TOP_DIR}/src/ExecuteProcess.cpp)

install(TARGETS execute_process
        DESTINATION ${TOP_DIR}/install)
# If you don't specify the DESTINATION, the following error is shown:
# "install TARGETS given no RUNTIME DESTINATION for executable target"
