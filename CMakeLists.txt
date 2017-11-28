cmake_minimum_required(VERSION 3.5)

add_executable(pandar_driver
	src/main.c
	src/pandar_driver.cpp
)

target_link_libraries(pandar_driver
	pthread
)

include_directories(
	include
)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g" )

install(PROGRAMS ${CMAKE_BINARY_DIR}/pandar_control DESTINATION bin)