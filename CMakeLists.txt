#cmake_minimum_required(VERSION 3.5)
cmake_minimum_required(VERSION 2.8)
project(pandoraSDK)
find_package( OpenCV REQUIRED )
find_package( Boost REQUIRED )
find_package(PCL REQUIRED )

set (CMAKE_CXX_STANDARD 11)
set (CMAKE_CXX_FLAGS "-g")
add_library(${PROJECT_NAME}
	src/pandoraSDK.cc
)

include_directories(
	include
	src/pandora_camera_client/include
	${Boost_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}

)

#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})


add_subdirectory(src/pandora_camera_client)
add_subdirectory(src/lib)
#add_subdirectory(test)

target_link_libraries(pandoraSDK
	pandar_input
	pandar_rawdata
	pthread
	pandora_client
	yaml-cpp
	${OpenCV_LIBS}
	${Boost_LIBRARIES}
	${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES}
)

add_executable(pandora_sdk_test 
	src/test.cc
)
target_link_libraries(pandora_sdk_test 
	${PROJECT_NAME}
)