cmake_minimum_required(VERSION 2.8)
project(hesaiLidarSDK)


find_package( Boost REQUIRED )
find_package(PCL REQUIRED )

set (LINK_LIBRARY_PLUS "")

option(Camera_Enable "Enable Camera Driver, For Pandora" OFF)
option(Debug_Enable "Enable Camera Driver, For Pandora" OFF)

set (CMAKE_CXX_FLAGS "-fPIC -std=c++11")
set (CMAKE_CXX_STANDARD 11)

set (SRC 
	src/calibration.cc
	src/input.cc
	src/rawdata.cc
	src/hesaiLidarSDK_IN.cc
	src/hesaiLidarSDK.cc
	)

add_subdirectory(src/pandora_camera_client)
if(Camera_Enable)
	message("Build Camera")
	find_package( OpenCV REQUIRED )
	add_definitions("-DHESAI_WITH_CAMERA")
	set (LINK_LIBRARY_PLUS "${OpenCV_LIBS}")
	set (JPEG_LIB "jpeg")
	set (SRC 
		${SRC}
		src/utilities.cc
		)
endif(Camera_Enable)

if(Debug_Enable)
	message("Build Debug")
	add_definitions("-DDebug")
	set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
endif(Debug_Enable)


include_directories(
	include
	src/pandora_camera_client/include
	${Boost_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
)


add_library( ${PROJECT_NAME} SHARED
	${SRC}
)

target_link_libraries(${PROJECT_NAME}
		pandora_client
		pcap
		yaml-cpp
	   	${JPEG_LIB}
		${Boost_LIBRARIES}
		${PCL_COMMON_LIBRARIES} 
		${PCL_IO_LIBRARIES}
		${LINK_LIBRARY_PLUS}
)

if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR})
	

	add_executable(pandora_sdk_test 
		src/test.cc
	)
	target_link_libraries(pandora_sdk_test 
		${PROJECT_NAME}
		${Boost_LIBRARIES}
		${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES}
		${LINK_LIBRARY_PLUS}
	)

endif(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR})


set(CMAKE_INSTALL_PREFIX ${CMAKE_BINARY_DIR}/SDK)

install(PROGRAMS include/hesaiLidarSDK.h DESTINATION include)
install(PROGRAMS include/point_types.h DESTINATION include)
install(DIRECTORY sample DESTINATION .)
install(PROGRAMS ${CMAKE_BINARY_DIR}/lib${PROJECT_NAME}.so DESTINATION lib)
