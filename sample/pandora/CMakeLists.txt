cmake_minimum_required(VERSION 2.8)

find_package(Boost REQUIRED )
find_package(PCL REQUIRED )
find_package( OpenCV REQUIRED )


set (CMAKE_CXX_STANDARD 11)

# When sdk compiled with Camera_Enable , This definition must be add to cmakelists
add_definitions("-DHESAI_WITH_CAMERA")

include_directories(
	${CMAKE_CURRENT_SOURCE_DIR}/../../include
	${Boost_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)


add_executable(pandora
	pandora.cc
)
target_link_libraries(pandora
	${CMAKE_CURRENT_SOURCE_DIR}/../../lib/libhesaiLidarSDK.so
	${Boost_LIBRARIES}
	${PCL_LIBS}
	${OpenCV_LIBS}
	jpeg
)

