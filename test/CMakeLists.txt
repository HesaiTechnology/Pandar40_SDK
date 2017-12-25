cmake_minimum_required(VERSION 2.8)
project(pandoarSDK_sample)

find_package( OpenCV REQUIRED )
find_package( Boost REQUIRED )
find_package(PCL REQUIRED )


set (CMAKE_CXX_STANDARD 11)


include_directories(
	include
	${Boost_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME} 
	test.cc
)


execute_process(COMMAND lsb_release -rs OUTPUT_VARIABLE OS_VERSION)
message (${OS_VERSION})

if(OS_VERSION EQUAL 14.04)
set(pandoraSDK 14.04/libpandoraSDK.so)

elseif(OS_VERSION EQUAL 16.04)
set(pandoraSDK 16.04/libpandoraSDK.so)

else()
message (wrong os_version)

endif()

message("${CMAKE_SOURCE_DIR}/lib/${pandoraSDK}")

target_link_libraries(${PROJECT_NAME} 
  ${CMAKE_SOURCE_DIR}/lib/${pandoraSDK}
	${OpenCV_LIBS}
	${Boost_LIBRARIES}
	${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES}
)