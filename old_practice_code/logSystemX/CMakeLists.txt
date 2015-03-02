cmake_minimum_required(VERSION 2.8)
add_definitions(-DBOOST_ALL_DYN_LINK)
set(CMAKE_CXX_FLAGS "-std=c++11 -g")

find_package(Boost 1.54.0 COMPONENTS log log_setup thread system unit_test_framework REQUIRED)
if(${Boost_FOUND})
	include_directories(${Boost_INCLUDE_DIRS})
	#add_executable(severityFormatTestX severityFormatTestX.cpp)
	add_executable(logSystemX boostTestXMain.cpp logSystemX.cpp) 
	target_link_libraries(logSystemX 
						  ${Boost_LOG_LIBRARY}
						  ${Boost_LOG_SETUP_LIBRARY}
						  ${Boost_THREAD_LIBRARY}
						  ${Boost_SYSTEM_LIBRARY}
						  ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY}
						 )
endif()

message(STATUS ${Boost_LIBRARIES})
