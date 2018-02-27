cmake_minimum_required(VERSION 2.8)
project(hesaiLidarSDK)

find_package( OpenCV REQUIRED )
find_package( Boost REQUIRED )
find_package(PCL REQUIRED )


set (CMAKE_CXX_STANDARD 11)
set (CMAKE_CXX_FLAGS "-g -fPIC")
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
)

add_executable(pandora_sdk_test 
	src/test.cc
)
target_link_libraries(pandora_sdk_test 
	${PROJECT_NAME}
)

add_executable(jpeg-test
	src/test-libjpeg.cc
)
target_link_libraries(jpeg-test
		${OpenCV_LIBS}
		jpeg
)