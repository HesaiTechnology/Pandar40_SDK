cmake_minimum_required(VERSION 2.8.3)
project(pandora_camera)

add_compile_options(-std=c++11)


add_executable(pandora_sdk_test 
	main.cc
)


INCLUDE_DIRECTORIES(
#	../include
)


target_link_libraries(pandora_sdk_test 
	pandoraSDK
)