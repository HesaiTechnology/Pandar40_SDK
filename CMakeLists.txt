cmake_minimum_required(VERSION 2.8)
project(hesaiLidarSDK)


find_package( Boost REQUIRED )
find_package(PCL REQUIRED )

set (LINK_LIBRARY_PLUS "")

option(Camera_Enable "Enable Camera Driver, For Pandora" OFF)
set (CMAKE_CXX_FLAGS "-g -fPIC")

if(Camera_Enable)
	message("Build Camera")
	find_package( OpenCV REQUIRED )
	add_definitions("-DHESAI_WITH_CAMERA")
	set (LINK_LIBRARY_PLUS ${OpenCV_LIBS})
endif(Camera_Enable)

set (CMAKE_CXX_STANDARD 11)

set (CMAKE_C_FLAGS "-g -fPIC")


include_directories(
	include
	src/pandora_camera_client/include
	${Boost_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
)

add_subdirectory(src/pandora_camera_client)
add_subdirectory(src/lib)

#link_directories(${PCL_LIBRARY_DIRS})

#add_subdirectory(test)

add_library( ${PROJECT_NAME} SHARED
	src/hesaiLidarSDK.cc
)

target_link_libraries(${PROJECT_NAME}
	pandora_sdk_in
	pcap
	yaml-cpp
	${Boost_LIBRARIES}
	${PCL_COMMON_LIBRARIES} 
	${PCL_IO_LIBRARIES}
	${LINK_LIBRARY_PLUS}
)

add_executable(pandora_sdk_test 
	src/test.cc
)
target_link_libraries(pandora_sdk_test 
	${PROJECT_NAME}
	${Boost_LIBRARIES}
	${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES}
	${LINK_LIBRARY_PLUS}
)
